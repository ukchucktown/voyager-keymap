#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  HSV_172_255_255,
  HSV_86_255_255,
  HSV_0_255_255,
  HSV_0_0_255,
  MAC_MISSION_CONTROL,
};



enum tap_dance_codes {
  DANCE_0,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_EQUAL,       KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,       
    KC_TILD,        KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,        
    KC_CAPS,        KC_A,           KC_S,           LT(2,KC_D),     LT(1,KC_F),     KC_G,                                           KC_H,           LT(1,KC_J),     LT(2,KC_K),     KC_L,           KC_SCLN,        KC_QUOTE,       
    KC_NO,          KC_Z,           MT(MOD_LCTL, KC_X),MT(MOD_LALT, KC_C),MT(MOD_LGUI, KC_V),KC_B,                                           KC_N,           MT(MOD_RGUI, KC_M),MT(MOD_RALT, KC_COMMA),MT(MOD_RCTL, KC_DOT),KC_SLASH,       KC_TAB,         
                                                    KC_ENTER,       MT(MOD_LSFT, KC_ESCAPE),                                MT(MOD_RSFT, KC_BSPC),KC_SPACE
  ),
  [1] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_AMPR,        KC_LPRN,        KC_RPRN,        KC_EXLM,        KC_AT,          KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_PIPE,        KC_LCBR,        KC_RCBR,        KC_HASH,        KC_DLR,         KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TILD,        KC_LBRC,        KC_RBRC,        KC_PERC,        KC_CIRC,        KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_BRIGHTNESS_DOWN,KC_BRIGHTNESS_UP,KC_AUDIO_MUTE,  KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,                                RALT(RGUI(RCTL(RSFT(KC_C)))),RALT(RGUI(RCTL(RSFT(KC_S)))),RALT(RGUI(RCTL(RSFT(KC_O)))),RALT(RGUI(RCTL(RSFT(KC_W)))),KC_LCBR,        KC_RCBR,        
    RGB_SLD,        RGB_SPI,        RGB_VAI,        RGB_SAI,        RGB_HUI,        TOGGLE_LAYER_COLOR,                                RALT(RCTL(RSFT(KC_S))),RALT(RCTL(RSFT(KC_P))),RALT(RCTL(RSFT(KC_N))),MAC_MISSION_CONTROL,KC_LBRC,        KC_RBRC,        
    RGB_MODE_FORWARD,RGB_SPD,        RGB_VAD,        RGB_SAD,        RGB_HUD,        RGB_TOG,                                        KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_Q,           TD(DANCE_0),    
    KC_TRANSPARENT, KC_TRANSPARENT, HSV_172_255_255,HSV_86_255_255, HSV_0_255_255,  HSV_0_0_255,                                    LALT(LCTL(LSFT(KC_H))),LALT(LCTL(LSFT(KC_J))),LALT(LCTL(LSFT(KC_K))),LALT(LCTL(LSFT(KC_L))),LALT(LCTL(LSFT(KC_SCLN))),RALT(RGUI(KC_H)),
                                                    KC_LEFT_GUI,    KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, MT(MOD_LGUI, KC_X),KC_TRANSPARENT, MT(MOD_LCTL, KC_V),KC_TRANSPARENT,                                 KC_TRANSPARENT, MT(MOD_RCTL, KC_M),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};


uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case LT(2,KC_D):
            return TAPPING_TERM -5;
        case LT(1,KC_F):
            return TAPPING_TERM -10;
        case KC_G:
            return TAPPING_TERM -5;
        case MT(MOD_LGUI, KC_V):
            return TAPPING_TERM -10;
        case MT(MOD_LSFT, KC_ESCAPE):
            return TAPPING_TERM -15;
        case KC_H:
            return TAPPING_TERM -5;
        case LT(1,KC_J):
            return TAPPING_TERM -10;
        case LT(2,KC_K):
            return TAPPING_TERM -5;
        case MT(MOD_RGUI, KC_M):
            return TAPPING_TERM -10;
        case MT(MOD_RSFT, KC_BSPC):
            return TAPPING_TERM -15;
        case TD(DANCE_0):
            return TAPPING_TERM -5;
        default:
            return TAPPING_TERM;
    }
}


bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case MAC_MISSION_CONTROL:
      HCS(0x29F);

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_172_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(172,255,255);
      }
      return false;
    case HSV_86_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(86,255,255);
      }
      return false;
    case HSV_0_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,255,255);
      }
      return false;
    case HSV_0_0_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,0,255);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[1];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
    }
    if(state->count > 3) {
        tap_code16(KC_TAB);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_TAB); break;
        case DOUBLE_TAP: register_code16(RSFT(KC_TAB)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TAB); register_code16(KC_TAB);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_TAB); break;
        case DOUBLE_TAP: unregister_code16(RSFT(KC_TAB)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TAB); break;
    }
    dance_state[0].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
};
