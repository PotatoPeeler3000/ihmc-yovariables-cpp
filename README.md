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

**Phase 6 done:** a mechanical GoogleTest port of the JUnit test suite, across all five directories it lives
in - `variable/` (6 files), `registry/` (7 files), `buffer/` (3 files, skipping
`YoBufferVariableEntryConcurrencyBenchmark`, a benchmark rather than a test), `parameters/` (8 files), and
`filters/` (18 files) - 385 tests total, all passing. The point of doing this mechanically rather than
selectively was exactly realized: three genuine bugs in earlier phases' production code surfaced purely from
porting tests that happened to exercise paths the phases' own smoke tests hadn't:

- **C++ name-hiding**: `YoBoolean`/`YoDouble`/`YoInteger`/`YoLong`/`YoEnum<E>` each declare a 2-arg
  `setValueFromDouble(double, bool)`/`setValueFromLongBits(int64_t, bool)`/`parseValue(string, bool)`
  override, which - per ordinary C++ name lookup, unlike Java's flat virtual dispatch - hides *all* base-class
  overloads of that name from the derived class's public interface, including `YoVariable`'s 1-arg convenience
  overloads. Fixed with `using YoVariable::setValueFromDouble;` (etc.) in each of the 5 classes.
- **`YoRegistry::removeVariable` segfault**: calling it directly on a variable actually registered there (as
  opposed to indirectly via `YoVariable::setRegistry(nullptr)`, the usual path) crashed. `setRegistry(nullptr)`
  re-enters `removeVariable` while the entry is still present, and that reentrant call erases
  `nameToVariableMap_`'s entry; the outer call then reused its now-invalidated iterator to erase the same entry
  again. Fixed by erasing by key instead, which - like Java's `Map.remove(key)`, the reason the equivalent
  reentrancy is harmless there - is idempotent.
- **`YoBoolean::parseValue` case sensitivity**: it compared the input string to `"true"` with `==`, so loading
  `"TRUE"` or `"True"` silently produced `false`. Java's equivalent uses `Boolean.parseBoolean`, documented
  case-insensitive. Fixed with a case-insensitive comparison.

Also discovered this way: `YoVariableList` (in the `registry` package) had never been carried over by Phases
1-5 - a real gap, not a test-only omission - and is now ported as production code
(`registry/yo_variable_list.h`/`.cpp`) rather than stubbed just for its test file.

Recurring adaptations, each commented in place at the specific test that needed them rather than applied
uniformly:

- Nullable-enum divergence (`YoEnum<E>::getEnumValue()` throws on a null current value rather than Java's
  nullable return, per Phase 1) shows up again in `EnumParameterTest`; the fix is the same - go through
  `getEnumValueOrNull()` on the backing variable instead of the parameter's own `getValue()`.
  `YoEnumTest.testEmptyConstantList`'s enum-backed half isn't ported: magic_enum can't reflect a genuinely
  empty enum (it needs at least one enumerator to calibrate the value range it probes), a real, previously
  unexercised capability gap against Java (which allows a zero-constant enum fine).
- A handful of tests exercise something with no C++ equivalent and are dropped with an explanatory comment
  rather than faked: reflection-only assertions (`YoVariableTest.testRecursiveCompareYoVariables`,
  `YoEnumTest.testGetEnumType`), a null-array-element case (`std::string` has no null state, unlike
  `String[]`), and several `catch (NullPointerException)` patterns where the Java code deliberately
  dereferences a null reference - not reproducible in C++ without undefined behavior.
- `YoBufferVariableEntry::writeBufferAt` is fully private in this port (Java's is package-private, reachable
  from the same-package test) - tests that used it directly now go through the public `writeIntoBufferAt` after
  setting the backing variable's value, which reaches the same buffer state.
- Where a Java test's tolerance implicitly relied on `Double.toString()`'s exact-round-trip guarantee (no
  delta, or `Double.MIN_VALUE`), the C++ equivalent constructs its input strings with enough precision (17
  significant digits, not `std::to_string`'s default 6) to round-trip exactly instead - `std::to_string` is a
  real, already-documented (Phase 1-4) formatting divergence, not something to fix here, just something the
  test's own string construction has to route around.
- `java.util.Random` usage ported to `std::mt19937`/`std::mt19937_64` with a fixed seed throughout, including
  the handful of Java tests that used an *unseeded* `Random()` - every assertion in this suite checks
  self-consistency or a statistical property, never a value tied to a specific seed's exact sequence, so a
  fixed seed changes nothing being tested and makes the C++ side strictly more reproducible.
- A few tests reach into package-private fields Java's access control permitted from a same-package test class
  (`GlitchFilteredYoBooleanTest.testCounter`'s direct read of the `counter` field) that this port made fully
  private with no accessor; these are dropped rather than approximated, since there's no public-API proxy that
  observes the same internal state precisely.

**Server Phases 1-4 done (branch `feature/yovariable-server`):** a C++ `YoVariableServer`
(`include/ihmc/robotDataLogger/`, new `ihmc_robot_data_logger` library target), wire-compatible with the real,
unmodified Java `YoVariableClient` from `ihmc-robot-data-logger` - not a port of `us.ihmc.yoVariables.*` like
everything above, but a from-scratch C++ implementation of `us.ihmc.robotDataLogger`'s network protocol (HTTP +
WebSocket + a hand-rolled CDR encoding + LZ4 block compression), verified byte-for-byte against that protocol's
actual Java source rather than reimplemented from a general description of it:

- **Phase 1 (`wire/`)**: `CDRBuffer` - the alignment/payload-header/endianness subset `CustomLogDataPublisherType`
  and `VariableChangeRequest` actually use - and vendored LZ4 (`third_party/lz4`, block API, not the frame
  format, to match `net.jpountz.lz4.LZ4Compressor`'s wire format).
- **Phase 2 (`handshake/`)**: `YoVariableHandShakeBuilder` replicates Java's exact registry-ID DFS numbering
  (ID 0 reserved for a synthetic root, real registries start at 1, a registry's own variables precede its
  children) and produces JSON matching `ROS2JSONSerializer`'s shape (root-wrapped under
  `"us::ihmc::robotDataLogger::<Type>"`, `YoType`/`LoadStatus` as enum-name strings) for `/handshake.json`.
- **Phase 3 (`http/`)**: a minimal HTTP/1.1 server (standalone Asio) covering exactly `/announcement.json`,
  `/handshake.json`, and the `/websocket` upgrade handoff - not a general-purpose HTTP library, since the real
  client only ever requests these fixed endpoints.
- **Phase 4 (`websocket/`, `yo_variable_server.*`)**: RFC6455 handshake/framing, `RegistrySendBuffer` +
  `encodeLogDataFrame`/`decodeLogDataFrame` (the `CustomLogDataPublisherType` equivalent), and the
  `YoVariableServer` public class itself (`setMainRegistry()`/`start()`/`update()`/`close()`, matching Java's
  shape).

**Verified against the real Java client, not just self-consistency**: `examples/simple_server_example.cpp`
publishes a `YoDouble`/`YoInteger`/`YoBoolean`/`YoLong`/`YoEnum` registry; a temporary harness using the actual
`us.ihmc.robotDataLogger.YoVariableClient` (built from the sibling `ihmc-robot-data-logger` checkout, no
modifications) connected to it over `localhost:8008` and received correct, internally-consistent live values
(e.g. `bigCounter` exactly `1000x` `counter` in every sample, `color`'s ordinal matching `counter % 3`), then
correctly detected disconnection when the C++ server shut down. This is the only test in the whole port that
actually crosses the C++/Java boundary; `YoVariableServerTest.testFullProtocolSelfRoundTrip` covers the same
path with a hand-rolled C++ client for fast, repeatable CI-style verification.

Real bugs found (both from concurrent close()/shutdown() races, not from the wire-format work itself - the wire
format bugs, like the alignment pad this surfaced between `CustomLogDataPublisherType`'s `type` and `registry`
fields, were caught and documented while implementing Phase 1, before any test ran green against a wrong
assumption):

- **`HttpServer`/`WebSocketConnection` UAF**: a detached per-connection thread could outlive its owning object,
  so a client that stayed connected past `stop()`/the destructor caused a use-after-free. Fixed by tracking every
  connection and joining its thread before `stop()` returns.
- **Closing a socket concurrently with another thread's in-flight `poll()` on the same fd is racy on macOS**
  (confirmed by sampling a hung test process mid-`poll()`): sometimes the close is observed, sometimes the poll
  never returns. Fixed by `shutdown(SHUT_RDWR)`-ing to unblock the read, then only actually `close()`-ing once
  that thread has been joined (via the last `shared_ptr<socket>` reference going away), so nothing is still
  polling the fd when it's closed.

Explicitly deferred scope, all documented in code where the gap is, not silently dropped: joints, YoGraphics,
`Summary`, `ReferenceFrameInformation` (handshake always emits Java-spec-correct empty/default values for
these); `/model.sdf`/`/resources.zip` (server always reports `hasModel=false`, which the real client already
special-cases to skip fetching them); inbound `VariableChangeRequest` handling, the `SEND_TIMESTAMPS`/UDP
timestamp channel, the text command/echo protocol, UDP multicast autodiscovery, disk logging, and a real
`reconnectKey` (stubbed - breaks `reconnect()` only, not first connect, since the client never validates it
before then).

Dependency note: the plan called for system OpenSSL for SHA-1 (WebSocket handshake) - dropped after
`find_package(OpenSSL)` failed on the dev machine (Homebrew's `openssl@3` is keg-only, not discoverable without
an extra CMake hint that can't be assumed on every machine). Vendored a small public-domain SHA-1 implementation
instead (`third_party/sha1`), consistent with how LZ4 is already vendored - the whole build now has zero system
package prerequisites, only `FetchContent`/vendored dependencies.

Not yet started: unit tests (mechanical GoogleTest port of the existing JUnit suite).
