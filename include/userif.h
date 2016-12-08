#ifndef _USERIF_
#define _USERIF_

#define RELOAD_MASK 0xf
#define LED_EN_MASK     0x10

enum {
  MV_LED,
  MV2_LED,
  MAN_LED,
  PRO_LED
};
void uiSetDispContent(uint8_t *c);
void uiSetDispFlash(uint8_t f);
void uiDispSetEnable(void);
void uiDispSetBrightness(uint8_t b);
void uiLedSet(uint8_t id, uint8_t freq);
void uiLedClear(uint8_t id);
void uiInit(void);
int16_t uiLedPoll(uint32_t ct);
int16_t uiKeyPoll(uint32_t ct);

#endif