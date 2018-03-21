#pragma once

#define QUOPA_FLIGHT_LIMIT 3000

typedef enum quopa_state_t
{
    QUOPA_INACTIVE = 0,
    QUOPA_INIT     = 1,
    QUOPA_ACTIVE   = 2,
    QUOPA_FAILED   = 3,
} quopa_state;

enum
{
    QUOPA_STYLE_AUTO        = 0,
    QUOPA_STYLE_SEMIAUTO    = 1,
    QUOPA_STYLE_MANUAL      = 2,
    QUOPA_STYLE_AUTO_2      = 3,
    QUOPA_STYLE_SEMIAUTO_2  = 4,
    QUOPA_STYLE_MANUAL_2    = 5,
} quopa_style;

extern volatile quopa_state quopaState;
extern volatile quopa_state dshotBeepState;

extern int InitDshotBeep(void);
extern int InitQuopaMode(void);
extern int HandleQuopaMode(void);
extern int HandleDshotBeep(void);