
#ifndef CUMO_EXPORT_H
#define CUMO_EXPORT_H

#ifdef CUMO_STATIC_DEFINE
#  define CUMO_EXPORT
#  define CUMO_NO_EXPORT
#else
#  ifndef CUMO_EXPORT
#    ifdef cumotion_cumotion_EXPORTS
        /* We are building this library */
#      define CUMO_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CUMO_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CUMO_NO_EXPORT
#    define CUMO_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CUMO_DEPRECATED
#  define CUMO_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CUMO_DEPRECATED_EXPORT
#  define CUMO_DEPRECATED_EXPORT CUMO_EXPORT CUMO_DEPRECATED
#endif

#ifndef CUMO_DEPRECATED_NO_EXPORT
#  define CUMO_DEPRECATED_NO_EXPORT CUMO_NO_EXPORT CUMO_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CUMO_NO_DEPRECATED
#    define CUMO_NO_DEPRECATED
#  endif
#endif
#ifdef _MSC_VER
#  define CUMO_EXTERN_TEMPLATE_EXPORT
#  define CUMO_TEMPLATE_EXPORT CUMO_EXPORT
#else
#  define CUMO_EXTERN_TEMPLATE_EXPORT CUMO_EXPORT
#  define CUMO_TEMPLATE_EXPORT
#endif

#endif /* CUMO_EXPORT_H */
