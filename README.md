# ihmc-yovariables-cpp

C++ port of [ihmc-yovariables](https://github.com/ihmcrobotics/ihmc-yovariables), ported one package at a time
against the Java source as the reference implementation.

## Scope

Not ported (for now): the `euclid` package (Euclid-typed `Yo*` wrappers, e.g. `YoPoint3D`, `YoQuaternion`) and
`math/YoMatrix.java` (EJML-backed) — both pull in external geometry/matrix libraries that are a separate scope
decision from the rest of the port.

## Design decisions carried over from the Java source

- **`YoVariable` dispatch**: the Java type set is open (the base class has a `public` constructor, meant to be
  subclassed) and the runtime hot path already erases every variable's value to `double` at the buffer boundary
  (`getValueAsDouble`/`setValueFromDouble`), so the C++ port uses a polymorphic base class (`virtual`), not
  `std::variant` — variant requires a closed, compile-time-fixed alternative list, which an open/extensible type
  set rules out. Ownership mirrors Java's reference semantics rather than introducing C++ ownership machinery:
  `YoVariable`/`YoRegistry` store non-owning raw pointers to each other, matching how real consumer code holds
  `YoVariable`s as long-lived members rather than registry-owned objects. `YoBuffer`, by contrast, does own its
  `YoBufferVariableEntry` instances outright (via `std::unique_ptr`) — they're internal bookkeeping objects with
  no other stated owner in the Java source, unlike `YoVariable` itself.
- **`YoEnum<E>`**: ported as a template (`YoEnum<E>` inheriting the non-template `YoVariable` base, same as any
  other subclass), using [magic_enum](https://github.com/Neargye/magic_enum) in place of Java's reflective
  `Class<E>.getEnumConstants()`.
- **Concurrency**: `YoBufferVariableEntry` mirrors the current (lock-free, atomics-based) version of the Java
  class, not the older `synchronized` version.

## Build

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## Status

**Phase 1 done:** variable types (`YoBoolean`, `YoDouble`, `YoInteger`, `YoLong`, `YoEnum<E>`), the 5
provider interfaces, `YoRegistry`/`YoNamespace`/`YoVariableHolder`, the listener and exception types, and
the subset of `YoTools`/`YoSearchTools` those depend on.

`YoRegistry` was folded into this phase alongside `YoVariable` (rather than the originally planned
"types + YoVariable" / "buffer + registry" split) because they're a genuine circular pair in the Java
source: `YoVariable.setRegistry()` calls back into `YoRegistry`'s methods directly. `YoBuffer` has no such
cycle - it only touches `YoVariable` through its already-abstract interface - so it remains its own,
separable next phase.

Known fidelity gaps from this phase, called out for when they start to matter: `YoDouble`'s
`getValueAsString()`/`toString()` use `std::to_string`/`snprintf` rather than reproducing Java's
`Double.toString()`/`Formatter` rules exactly; `YoEnum<E>::getValue()` throws when the current value is
null (Java can return a null reference there) - use `getEnumValueOrNull()` for the nullable form; and
`YoTools`'s diagnostic-printing helpers (`printStatistics`/`getRegistryInfo`) are deferred to the
utilities phase, along with the `parameters`-list bookkeeping in `YoRegistry` (present and correct, but
inert until `YoParameter` exists to override `isParameter()`/`getParameter()`).

**Phase 2 done:** `buffer/interfaces/` (the 8 reader/listener/processor interfaces), `YoBufferBounds`,
`YoBufferVariableEntry`, `KeyPointsHandler`, `YoBuffer`.

`YoBufferVariableEntry`'s lock-free design translates `AtomicLongArray` (Java's workaround for having no
`AtomicDoubleArray`) to a plain array of `std::atomic<double>` (C++ needs no bit-reinterpretation trick), and
`AtomicReference<YoBufferBounds>` + CAS to `std::shared_ptr<const YoBufferBounds>` managed via the C++17
`std::atomic_load`/`atomic_compare_exchange_weak` free functions — chosen over a raw atomic pointer specifically
because readers genuinely dereference `currentBounds_` concurrently with writer CAS swaps by design (mirroring
the real benchmarked design from the Java source), and only `shared_ptr`'s reference counting gives safe
reclamation without a GC. The whole-array swap on resize (`bufferData_`, a raw `std::atomic<AtomicDoubleArray*>`)
uses simple immediate deletion instead, since resize operations are documented single-thread-only in both the
Java source and this port - the difference is that Java's GC quietly covers a violation of that contract where
C++ would not. Verified against the concurrent write/read pattern with ThreadSanitizer (5 runs, clean) in
addition to the regular test suite.

Known adaptations from this phase: `getBounds()`/`getWindowBounds()`/`getCustomBounds()` return
`YoBufferBounds` by value rather than by reference, since a reference into the atomically-swapped bounds
storage isn't safe to hand back to a caller in C++ the way Java's GC-backed reference is; `YoBuffer::getEntry()`
covariantly overrides its interface's return type (`YoBufferVariableEntry*` for `YoBufferVariableEntryReader*`)
using plain C++ pointer covariance, unlike `YoRegistry::getChildren()`'s container-covariance workaround in
Phase 1.

**Phase 3 done:** `parameters/` in full — `YoParameter` and the 5 concrete types (`BooleanParameter`,
`DoubleParameter`, `IntegerParameter`, `LongParameter`, `EnumParameter<E>`), `ParameterData`,
`AbstractParameterReader`/`Writer`, `DefaultParameterReader`, `SingleParameterReader`, `XmlParameterReader`/
`Writer` (via [pugixml](https://github.com/zeux/pugixml), FetchContent'd like magic_enum), the `parameters/xml`
token classes, and `YoParameterChangedListener`.

Each concrete parameter type mirrors the Java source's private-inner-class pattern (e.g. `BooleanParameter`'s
`YoBooleanParameter extends YoBoolean`) as an explicit nested `BackingVariable` holding a back-reference to its
owning parameter, since C++ has no implicit outer-instance pointer for nested classes. Two real bugs surfaced
and got fixed while building this, both worth knowing about:

- **Virtual dispatch during base-class construction.** `BackingVariable`'s constructor used to pass the real
  registry straight to its `YoBoolean`/`YoDouble`/etc. base constructor, which registers with the registry and
  triggers calls to `isParameter()`/`getParameter()`. In Java that already dispatches to the most-derived
  override during construction; in C++, a base class subobject under construction dispatches virtuals using its
  *own* vtable, not the eventually-most-derived one — so these calls silently resolved to `YoVariable`'s
  un-overridden defaults, and no parameter ever made it into `YoRegistry::parameters_`. Fixed by passing
  `nullptr` to the base constructor and calling `setRegistry(registry)` from `BackingVariable`'s own constructor
  *body*, after the derived vtable is active.
- **`duplicate()`'s ownership.** `YoVariable::duplicate()` returns `std::unique_ptr<YoVariable>`, but a
  duplicated parameter's returned variable and its new "shell" parameter object are supposed to be able to
  reference each other indefinitely (`getParameter()` on the former still working) — a relationship Java
  expresses for free via GC and a fixed single-owner `unique_ptr` return type cannot express directly. Resolved
  by deliberately leaking the shell (documented at each call site): the shell is never deleted, so its own
  `value_` member's destructor never runs, and the `unique_ptr` handed back to the caller remains the only thing
  that will ever actually free the backing variable. `duplicate()` is not on any hot path, so a bounded one-shell
  leak per call was judged preferable to a wider interface change or unsafe shared ownership.

Also added while here: `YoEnum<E>::getEnumValues()`, missed in Phase 1 (Java has it; only
`getEnumValuesAsString()` had been ported).

**Phase 4 done:** the rest of `tools/` — `YoFactories`, `YoGeometryNameTools`, and the diagnostic-printing
pieces of `YoTools` (`printStatistics`, `getRegistryInfo`, `toShortName`) that Phase 1 deferred.
`YoGeometryNameTools` turned out not to need excluding despite importing Euclid types: those imports are for
javadoc `@link`s only, never touched by any actual method signature.

`YoFactories::findOrCreateRegistry`/`createChainOfRegistries` needed the same kind of ownership adaptation as
`duplicate()` in Phase 3, for the same underlying reason: they can create new `YoRegistry`s that Java keeps
alive implicitly (reachable from the tree, via GC) with no explicit owner anywhere. Rather than leaking (as
Phase 3's `duplicate()` does, since that's a rarely-called leaf capability), these take an explicit
`std::vector<std::unique_ptr<YoRegistry>>& newRegistriesOwnershipSink` output parameter that any newly created
registries are appended to — making explicit what Java leaves implicit, without leaking, since a registry
tree built this way (unlike a one-off `duplicate()` call) is plausibly built once and kept for a program's
whole lifetime.

**Phase 5 done:** the `ihmc-yovariables-filters` Gradle module's non-Euclid, non-EJML classes - 28 files under
`filters/` (`AlphaFilteredYoVariable`, `RateLimitedYoVariable`, `GlitchFilteredYoBoolean`,
`SecondOrderFilteredYoDouble`, etc.; the full list is in `include/ihmc/yovariables/filters/`). Out of scope,
per the same reasoning as `euclid/`/`YoMatrix` earlier: all 26 files under the module's `euclid/filters/`
subpackage (they operate on Euclid-typed `YoFrame*` wrappers) and `filters/AlphaFilteredYoMatrix.java` (directly
EJML-backed). One more file imported EJML's `DMatrixRMaj` without needing it - see below.

Two structural issues came up repeatedly across this phase, both already established patterns from earlier
phases, reapplied here at volume:

- **Java's `new YoDouble(...)` factory helpers don't translate to a function returning a `YoVariable` by
  value** (`YoVariable` and its subclasses are neither copyable nor movable, matching their non-owning-pointer
  design from Phase 1). `VariableTools` - which Java uses to construct and return auxiliary YoVariables like a
  filter's internal alpha or window-size variable - became naming-convention helper functions instead
  (returning just the string name to use), with each filter class constructing its own auxiliary variables
  directly as members. Where a filter needs to support *either* an internally-owned `YoDouble` *or* an
  externally-provided `DoubleProvider` (both are common across this package, e.g. a fixed alpha value vs. an
  external alpha provider), the field is a `DoubleProvider*` pointer alongside a `std::optional<YoDouble>` (or
  `std::optional<ConstantDoubleProvider>`, a small new adapter class for wrapping a fixed double as a
  `DoubleProvider`, C++'s equivalent of Java's `() -> dt` lambda) that the pointer points into when owned
  internally.
- **Trivial external-library calls needed local substitutes**, since none of `us.ihmc.commons.{AngleTools,
  MathTools}`, `us.ihmc.euclid.tools.EuclidCoreTools`, or `us.ihmc.commons.DeadbandTools` are part of
  ihmc-yovariables (and so aren't otherwise being ported): a new `filters/filter_math.h` provides `clamp`,
  `interpolate`, `angleDifferenceMinusPiToPi`, and `applyDeadband` equivalents. `applyDeadband` in particular
  is a best-effort standard implementation, not a verified port, since `DeadbandTools`'s actual source wasn't
  available to check against.

`SimpleMovingAverageFilteredYoVariable` imports EJML's `DMatrixRMaj` but only ever uses it as a resizable
single-column buffer (`.get(i,0)`/`.set(i,0,v)`/`.reshape(n,1)`, no actual matrix math), so it didn't need
excluding alongside `AlphaFilteredYoMatrix` - it's ported using `std::vector<double>` instead. One behavioral
adaptation there: Java's `reset()` calls EJML's `reshape()`, whose exact retained-values-on-resize behavior is
underdocumented/implementation-specific; this port always zero-fills on reset (arguably more correct for a
filter reset than relying on that ambiguity, but worth knowing about).

One deliberately-not-virtual fidelity gap: `GlitchFilteredYoBoolean`/`GlitchFilteredYoInteger` override
`YoBoolean::set`/`YoInteger::set`, which are ordinary (non-virtual) methods in this port (Phase 1 didn't make
them virtual, matching that no other Phase 1-4 code needed it). Java's universal virtual dispatch means this
override is transparent regardless of the reference's static type; here, calling `set()` through a base
`YoBoolean&`/`YoInteger&` reference to one of these bypasses the glitch-filtering logic. Calling it through the
concrete `GlitchFilteredYo*` type (the normal way these are used) is unaffected.

`ihmc-yovariables-filters` also has 2 EJML-touching files skipped above and 26 Euclid-dependent files under
`euclid/filters/`, both permanently out of scope per the standing Euclid/EJML exclusions.

Not yet started: unit tests (mechanical GoogleTest port of the existing JUnit suite).
