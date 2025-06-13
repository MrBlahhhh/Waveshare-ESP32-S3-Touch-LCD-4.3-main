SET(SOURCES components/ui_comp_imgbutton1.c
    components/ui_comp.c
    screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_rpmbackground_png.c
    images/ui_img_rpmfront_png.c
    images/ui_img_rpmdial_png.c
    images/ui_img_checkengine_png.c
    images/ui_img_watertemp_png.c
    images/ui_img_turnsignal_png.c
    images/ui_img_parkingbrake_png.c
    images/ui_img_322054780.c
    images/ui_img_fuel_png.c)

add_library(ui ${SOURCES})
