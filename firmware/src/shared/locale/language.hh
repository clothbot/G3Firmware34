#ifndef LANGUAGE_HH_
#define LANGUAGE_HH_

#ifndef LANGUAGE
#define LANGUAGE ENGLISH
#endif

#if (LANGUAGE == ENGLISH)
#include "english.hh"
#endif

#if (LANGUAGE == FRANCAIS)
#include "francais.hh"
#endif

#endif
