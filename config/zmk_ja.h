#pragma once
#include <dt-bindings/zmk/keys.h>

/* Japanese JIS Layout Aliases for ZMK */

#define JA_N1    N1              // 1 !
#define JA_N2    N2              // 2 "
#define JA_N3    N3              // 3 #
#define JA_N4    N4              // 4 $
#define JA_N5    N5              // 5 %
#define JA_N6    N6              // 6 &
#define JA_N7    N7              // 7 '
#define JA_N8    N8              // 8 (
#define JA_N9    N9              // 9 )
#define JA_N0    N0              // 0 (No Symbol)

#define JA_MINUS MINUS           // - =
#define JA_EQL   LS(MINUS)       // = (Shift + -)
#define JA_CARET EQUAL           // ^ ~
#define JA_TILDE LS(EQUAL)       // ~ (Shift + ^)
#define JA_YEN   INT3            // ¥ |
#define JA_PIPE  LS(INT3)        // | (Shift + ¥)

#define JA_AT    LBKT            // @ `
#define JA_GRAVE LS(LBKT)        // ` (Shift + @)
#define JA_LBKT  RBKT            // [ {
#define JA_LBRC  LS(RBKT)        // { (Shift + [)
#define JA_RBKT  NONUS_HASH      // ] }
#define JA_RBRC  LS(NONUS_HASH)  // } (Shift + ])

#define JA_SEMI  SEMI            // ; +
#define JA_PLUS  LS(SEMI)        // + (Shift + ;)
#define JA_COLON SQT             // : *
#define JA_ASTRK LS(SQT)         // * (Shift + :)

#define JA_COMM  COMMA           // , <
#define JA_LT    LS(COMMA)       // < (Shift + ,)
#define JA_DOT   DOT             // . >
#define JA_GT    LS(DOT)         // > (Shift + .)
#define JA_SLASH SLASH           // / ?
#define JA_QUES  LS(SLASH)       // ? (Shift + /)
#define JA_BSLS  INT1            // \ _
#define JA_UNDS  LS(INT1)        // _ (Shift + \)

#define JA_LPAR  LS(N8)          // ( (Shift + 8)
#define JA_RPAR  LS(N9)          // ) (Shift + 9)
#define JA_QUOT  LS(N7)          // ' (Shift + 7)
#define JA_DQUOT LS(N2)          // " (Shift + 2)
#define JA_HASH  LS(N3)          // # (Shift + 3)
#define JA_DLLR  LS(N4)          // $ (Shift + 4)
#define JA_PRCNT LS(N5)          // % (Shift + 5)
#define JA_AMPS  LS(N6)          // & (Shift + 6)
