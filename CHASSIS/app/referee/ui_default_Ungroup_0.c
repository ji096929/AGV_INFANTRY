//
// Created by RM UI Designer
//

#include "ui_default_Ungroup_0.h"

#define FRAME_ID 1
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Ungroup_0;
ui_interface_line_t *ui_default_Ungroup_NewLine = (ui_interface_line_t *)&(ui_default_Ungroup_0.data[0]);

void _ui_init_default_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Ungroup_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Ungroup_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Ungroup_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Ungroup_0.data[i].operate_tpyel = 0;
    }

    ui_default_Ungroup_NewLine->figure_tpye = 0;
    ui_default_Ungroup_NewLine->layer = 0;
    ui_default_Ungroup_NewLine->start_x = 210;
    ui_default_Ungroup_NewLine->start_y = 327;
    ui_default_Ungroup_NewLine->end_x = 1390;
    ui_default_Ungroup_NewLine->end_y = 903;
    ui_default_Ungroup_NewLine->color = 1;
    ui_default_Ungroup_NewLine->width = 1;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_0, sizeof(ui_default_Ungroup_0));
}

void _ui_update_default_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_0, sizeof(ui_default_Ungroup_0));
}

void _ui_remove_default_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_0, sizeof(ui_default_Ungroup_0));
}
