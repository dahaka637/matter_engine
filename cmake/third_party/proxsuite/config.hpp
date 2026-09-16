#pragma once

// ProxSuite 0.7.3 é consumido como biblioteca header-only. O CMake upstream
// gera este arquivo apenas para versão/visibilidade; mantê-lo local evita
// trazer bindings, benchmarks e o framework CMake de robótica para a engine.
#define PROXSUITE_VERSION "0.7.3"
#define PROXSUITE_MAJOR_VERSION 0
#define PROXSUITE_MINOR_VERSION 7
#define PROXSUITE_PATCH_VERSION 3
#define PROXSUITE_VERSION_AT_LEAST(major, minor, patch) \
    (PROXSUITE_MAJOR_VERSION > (major) \
        || (PROXSUITE_MAJOR_VERSION >= (major) \
            && (PROXSUITE_MINOR_VERSION > (minor) \
                || (PROXSUITE_MINOR_VERSION >= (minor) \
                    && PROXSUITE_PATCH_VERSION >= (patch)))))
#define PROXSUITE_VERSION_AT_MOST(major, minor, patch) \
    (PROXSUITE_MAJOR_VERSION < (major) \
        || (PROXSUITE_MAJOR_VERSION <= (major) \
            && (PROXSUITE_MINOR_VERSION < (minor) \
                || (PROXSUITE_MINOR_VERSION <= (minor) \
                    && PROXSUITE_PATCH_VERSION <= (patch)))))

#if defined(_WIN32)
#define PROXSUITE_DLLIMPORT __declspec(dllimport)
#define PROXSUITE_DLLEXPORT __declspec(dllexport)
#define PROXSUITE_DLLLOCAL
#else
#define PROXSUITE_DLLIMPORT __attribute__((visibility("default")))
#define PROXSUITE_DLLEXPORT __attribute__((visibility("default")))
#define PROXSUITE_DLLLOCAL __attribute__((visibility("hidden")))
#endif

#define PROXSUITE_DLLAPI
#define PROXSUITE_LOCAL
#define PROXSUITE_EXPLICIT_INSTANTIATION_DECLARATION extern template
#define PROXSUITE_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI extern template
#define PROXSUITE_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
