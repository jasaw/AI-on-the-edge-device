#pragma once

#ifndef CFINDTEMPLATE_H
#define CFINDTEMPLATE_H

#include "CImageBasis.h"

struct RefInfo {
    std::string image_file; 
    int target_x = 0;
    int target_y = 0;
    int width = 0;
    int height = 0;
    int found_x;
    int found_y;
    int search_x;
    int search_y;
    int fastalg_x = -1;
    int fastalg_y = -1;
    int fastalg_min = -256;
    float fastalg_avg = -1;
    int fastalg_max = -1;
    float fastalg_SAD = -1;
    float fastalg_SAD_criteria = -1;
    int alignment_algo = 0;             // 0 = "Default" (nur R-Kanal), 1 = "HighAccuracy" (RGB-Kanal), 2 = "Fast" (1.x RGB, dann isSimilar), 3 = "HighAccuracy shift-only", 4 = "Fast shift-only", 5 = "off"
};




class CFindTemplate : public CImageBasis
{
    public:
        int tpl_width, tpl_height, tpl_bpp;    
        CFindTemplate(std::string name, uint8_t* _rgb_image, int _channels, int _width, int _height, int _bpp) : CImageBasis(name, _rgb_image, _channels, _width, _height, _bpp) {};

        bool FindTemplate(RefInfo *_ref);

        bool CalculateSimularities(uint8_t* _rgb_tmpl, int _startx, int _starty, int _sizex, int _sizey, int &min, float &avg, int &max, float &SAD, float _SADold, float _SADcrit);

        bool FindTemplateAlt(struct RefInfo *ref);
        void normalize_img(unsigned char *img, int _width, int _channels, 
                           int work_x_start, int work_x_stop, int work_y_start, int work_y_stop);
        uint8_t scan_find_template(struct RefInfo *ref, uint8_t* rgb_template);
};

#endif //CFINDTEMPLATE_H