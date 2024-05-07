#pragma once

#define ModualErrorCode(MODULAR, SUBMODULAR, CODE) \
    (1 << 30 ) | (MODULAR << 26) | (SUBMODULAR << 16) | CODE

