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

Ported so far: nothing yet — repo scaffold only. See the port plan for phase ordering.
