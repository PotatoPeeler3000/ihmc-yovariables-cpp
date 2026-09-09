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

Next: any other main classes (`parameters/`, `listener/` remainder, `exceptions/` remainder) — flagging again
that the Java build also has a separate `ihmc-yovariables-filters` module (56 files, 2 touching EJML) not yet
assigned to a phase.

Not yet started: utilities/helpers (`tools/` diagnostics), unit tests (mechanical GoogleTest port of the
existing JUnit suite).
