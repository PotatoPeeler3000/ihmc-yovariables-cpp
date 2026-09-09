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
  (`getValueAsDouble`/`setValueFromDouble`), so the C++ port uses a polymorphic base class
  (`virtual`, stored behind `std::shared_ptr`), not `std::variant` — variant requires a closed, compile-time-fixed
  alternative list, which an open/extensible type set rules out.
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

Next: `YoBuffer` (`YoBufferBounds`, `YoBufferVariableEntry`, `YoBuffer`, `KeyPointsHandler`).
