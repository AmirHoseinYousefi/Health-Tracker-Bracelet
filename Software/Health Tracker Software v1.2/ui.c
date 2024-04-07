// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.1.1
// LVGL VERSION: 8.3.3
// PROJECT: Sample

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t * ui_Screen1;
void ui_event_Screen1_Arc2(lv_event_t * e);
lv_obj_t * ui_Screen1_Arc2;
lv_obj_t * ui_Screen1_Label1;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 32
    #error "LV_COLOR_DEPTH should be 32bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Screen1_Arc2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_VALUE_CHANGED) {
        _ui_arc_set_text_value(ui_Screen1_Label1, target, "", " %");
    }
}

///////////////////// SCREENS ////////////////////
void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen1_Arc2 = lv_arc_create(ui_Screen1);
    lv_obj_set_width(ui_Screen1_Arc2, 230);
    lv_obj_set_height(ui_Screen1_Arc2, 230);
    lv_obj_set_x(ui_Screen1_Arc2, 0);
    lv_obj_set_y(ui_Screen1_Arc2, 3);
    lv_obj_set_align(ui_Screen1_Arc2, LV_ALIGN_CENTER);
    lv_arc_set_range(ui_Screen1_Arc2, 0, 270);
    lv_arc_set_value(ui_Screen1_Arc2, 0);

    lv_obj_set_style_arc_color(ui_Screen1_Arc2, lv_color_hex(0xF90404), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_Screen1_Arc2, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Screen1_Arc2, lv_color_hex(0xF90404), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1_Arc2, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_Screen1_Label1 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Screen1_Label1, LV_SIZE_CONTENT);   /// 10
    lv_obj_set_height(ui_Screen1_Label1, LV_SIZE_CONTENT);    /// 5
    lv_obj_set_x(ui_Screen1_Label1, 0);
    lv_obj_set_y(ui_Screen1_Label1, 90);
    lv_obj_set_align(ui_Screen1_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen1_Label1, "0 %");

    lv_obj_add_event_cb(ui_Screen1_Arc2, ui_event_Screen1_Arc2, LV_EVENT_ALL, NULL);

}

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    lv_disp_load_scr(ui_Screen1);
}
