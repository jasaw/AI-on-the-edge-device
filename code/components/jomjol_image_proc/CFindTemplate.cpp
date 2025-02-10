#include "CFindTemplate.h"

#include "ClassLogFile.h"
#include "Helper.h"
#include "../../include/defines.h"

#include <esp_log.h>

static const char* TAG = "C FIND TEMPL";

// #define DEBUG_DETAIL_ON  

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))


template<typename T> inline const T abs(T const & x)
{
    return ( x<0 ) ? -x : x;
}

uint8_t CFindTemplate::scan_find_template(struct RefInfo *ref, uint8_t* rgb_template)
{
    if (channels != 3)
        return 0;

    const uint64_t critSAD = 2000;
    int ow_start, ow_stop;
    int oh_start, oh_stop;
    unsigned char *img = rgb_image;

    ref->found_x = -1;
    ref->found_y = -1;

    if (ref->search_x == 0)
        ref->search_x = width;

    if (ref->search_y == 0)
        ref->search_y = height;

    ow_start = MAX(ref->target_x - ref->search_x, 0);
    ow_stop = MIN(ref->target_x + ref->search_x, width - tpl_width);
    oh_start = MAX(ref->target_y - ref->search_y, 0);
    oh_stop = MIN(ref->target_y + ref->search_y, height - tpl_height);

    // find best match
    uint64_t SAD = 0;
    uint64_t avgSAD = 0;
    uint64_t tmpMinSAD = tpl_width * tpl_height * channels * 255;
    uint64_t minSAD = tmpMinSAD * tmpMinSAD;
    uint64_t minSADvariance = minSAD;

    ESP_LOGD(TAG, "Scan finding ...");

    ESP_LOGD(TAG, "width %d, height %d, tpl_width %d, tpl_height %d", width, height, tpl_width, tpl_height);
    ESP_LOGD(TAG, "ow_start %d, ow_stop %d, oh_start %d, oh_stop %d, minSAD 0x%x%08x, minSADvariance 0x%x%08x",
            ow_start, ow_stop, oh_start, oh_stop,
            (unsigned int)(minSAD >> 32), (unsigned int)(minSAD & 0xffffffff),
            (unsigned int)(minSADvariance >> 32), (unsigned int)(minSADvariance & 0xffffffff));

    int xouter, youter, tpl_x, tpl_y;
    for (youter = oh_start; youter < oh_stop; youter++)
    {
        for (xouter = ow_start; xouter < ow_stop; xouter++)
        {
            SAD = 0;
            uint64_t SADvariance = 0;
            uint64_t prevDD = 0;
            for (tpl_y = 0; tpl_y < tpl_height; tpl_y++)
            {
                for (tpl_x = 0; tpl_x < tpl_width; tpl_x++)
                {
                    int pos_x = xouter + tpl_x;
                    int pos_y = youter + tpl_y;
                    stbi_uc* p_org = img + (channels * (pos_y * width + pos_x));
                    const stbi_uc* p_tpl = rgb_template + (channels * (tpl_y * tpl_width + tpl_x));
                    int greyscale_tpl = (int)p_tpl[0] + (int)p_tpl[1] + (int)p_tpl[2];
                    int greyscale_org = (int)p_org[0] + (int)p_org[1] + (int)p_org[2];
                    int diff = greyscale_tpl - greyscale_org;
                    int dd = diff * diff;
                    SADvariance += abs<int64_t>(diff - prevDD);
                    prevDD = diff;
                    SAD += dd;
                    if ((SAD >= minSAD) || (SADvariance >= minSADvariance))
                    {
                        // end loop
                        tpl_y = tpl_height;
                        tpl_x = tpl_width;
                    }
                }
            }
            if ((SAD < minSAD) && (SADvariance < minSADvariance))
            {
                minSAD = SAD;
                minSADvariance = SADvariance;
                ref->found_x = xouter;
                ref->found_y = youter;
            }
        }
    }

    avgSAD = minSAD / (tpl_width * tpl_height);

    ESP_LOGI(TAG, "Best match: SAD 0x%x%08x, avgSAD: 0x%x%08x, SADvariance: 0x%x%08x, found (x %d, y %d)",
            (unsigned int)(minSAD >> 32), (unsigned int)(minSAD & 0xffffffff),
            (unsigned int)(avgSAD >> 32), (unsigned int)(avgSAD & 0xffffffff),
            (unsigned int)(minSADvariance >> 32), (unsigned int)(minSADvariance & 0xffffffff),
            ref->found_x, ref->found_y);    

    ref->fastalg_x = ref->found_x;
    ref->fastalg_y = ref->found_y;
    ref->fastalg_min = minSAD;
    ref->fastalg_avg = avgSAD;
    ref->fastalg_max = tmpMinSAD * tmpMinSAD;
    ref->fastalg_SAD = SAD;

    return avgSAD < critSAD;
}

bool CFindTemplate::FindTemplateAlt(struct RefInfo *ref)
{
    uint8_t* rgb_template;

    if (file_size(ref->image_file.c_str()) == 0) {
        LogFile.WriteToFile(ESP_LOG_ERROR, TAG, ref->image_file + " is empty!");
        return false;
    }
   
    rgb_template = stbi_load(ref->image_file.c_str(), &tpl_width, &tpl_height, &tpl_bpp, channels);

    if (rgb_template == NULL) {
        LogFile.WriteToFile(ESP_LOG_ERROR, TAG, "Failed to load " + ref->image_file + "! Is it corrupted?");
        return false;
    }

    if (ref->search_x == 0)
        ref->search_x = width;
    if (ref->search_y == 0)
        ref->search_y = height;

    int ow_start, ow_stop;
    int oh_start, oh_stop;

    ow_start = MAX(ref->target_x - ref->search_x, 0);
    ow_stop = MIN(ref->target_x + ref->search_x + tpl_width, width);
    oh_start = MAX(ref->target_y - ref->search_y, 0);
    oh_stop = MIN(ref->target_y + ref->search_y + tpl_height, height);

    RGBImageLock();

    NormalizeImg(rgb_template, tpl_width, channels, 0, tpl_width, 0, tpl_height);
    NormalizeImg(rgb_image, width, channels, ow_start, ow_stop, oh_start, oh_stop);

    ESP_LOGD(TAG, "Finding template ...");

    bool found = scan_find_template(ref, rgb_template);

    RGBImageRelease();
    stbi_image_free(rgb_template);

    return found;
}

bool CFindTemplate::FindTemplate(RefInfo *_ref)
{
    uint8_t* rgb_template;

    if (file_size(_ref->image_file.c_str()) == 0) {
        LogFile.WriteToFile(ESP_LOG_ERROR, TAG, _ref->image_file + " is empty!");
        return false;
    }
   
    rgb_template = stbi_load(_ref->image_file.c_str(), &tpl_width, &tpl_height, &tpl_bpp, channels);

    if (rgb_template == NULL) {
        LogFile.WriteToFile(ESP_LOG_ERROR, TAG, "Failed to load " + _ref->image_file + "! Is it corrupted?");
        return false;
    }

//    ESP_LOGD(TAG, "FindTemplate 01");

    int ow, ow_start, ow_stop;
    int oh, oh_start, oh_stop;

    if (_ref->search_x == 0)
    {
        _ref->search_x = width;
        _ref->found_x = 0;
    }

    if (_ref->search_y == 0)
    {
        _ref->search_y = height;
        _ref->found_y = 0;
    }


    ow_start = _ref->target_x - _ref->search_x;
    ow_start = std::max(ow_start, 0);
    ow_stop = _ref->target_x + _ref->search_x;
    if ((ow_stop + tpl_width) > width)
        ow_stop = width - tpl_width;
    ow = ow_stop - ow_start + 1;

    oh_start = _ref->target_y - _ref->search_y;
    oh_start = std::max(oh_start, 0);
    oh_stop = _ref->target_y + _ref->search_y;
    if ((oh_stop + tpl_height) > height)
        oh_stop = height - tpl_height;
    oh = oh_stop - oh_start + 1;

    float avg, SAD;
    int min, max;
    bool isSimilar = false;

//    ESP_LOGD(TAG, "FindTemplate 02");

    if ((_ref->alignment_algo == 2) && (_ref->fastalg_x > -1) && (_ref->fastalg_y > -1))     // für Testzwecke immer Berechnen
    {
        isSimilar = CalculateSimularities(rgb_template, _ref->fastalg_x, _ref->fastalg_y, ow, oh, min, avg, max, SAD, _ref->fastalg_SAD, _ref->fastalg_SAD_criteria);
/*#ifdef DEBUG_DETAIL_ON
        std::string zw = "\t" + _ref->image_file + "\tt1_x_y:\t" + std::to_string(_ref->fastalg_x) + "\t" + std::to_string(_ref->fastalg_y);
        zw = zw + "\tpara1_found_min_avg_max_SAD:\t" + std::to_string(min) + "\t" + std::to_string(avg) + "\t" + std::to_string(max) + "\t"+ std::to_string(SAD);
        LogFile.WriteToDedicatedFile("/sdcard/alignment.txt", zw);
#endif*/
    }

//    ESP_LOGD(TAG, "FindTemplate 03");


    if (isSimilar)
    {
#ifdef DEBUG_DETAIL_ON  
        LogFile.WriteToFile(ESP_LOG_INFO, TAG, "Use FastAlignment sucessfull");
#endif
        _ref->found_x = _ref->fastalg_x;
        _ref->found_y = _ref->fastalg_y;

        stbi_image_free(rgb_template);
        
        return true;
    }

//    ESP_LOGD(TAG, "FindTemplate 04");


    double aktSAD;
    double minSAD = pow(tpl_width * tpl_height * 255, 2);

    RGBImageLock();

//    ESP_LOGD(TAG, "FindTemplate 05");
    int xouter, youter, tpl_x, tpl_y, _ch;
    int _anzchannels = channels;
    if (_ref->alignment_algo == 0)  // 0 = "Default" (nur R-Kanal)
        _anzchannels = 1;

    for (xouter = ow_start; xouter <= ow_stop; xouter++)
        for (youter = oh_start; youter <= oh_stop; ++youter)
        {
            aktSAD = 0;
            for (tpl_x = 0; tpl_x < tpl_width; tpl_x++)
                for (tpl_y = 0; tpl_y < tpl_height; tpl_y++)
                {
                    stbi_uc* p_org = rgb_image + (channels * ((youter + tpl_y) * width + (xouter + tpl_x)));
                    stbi_uc* p_tpl = rgb_template + (channels * (tpl_y * tpl_width + tpl_x));
                    if (channels == 3)
                    {
                        double greyscale_tpl = 0.2989 * p_tpl[0] + 0.5870 * p_tpl[1] + 0.1140 * p_tpl[2];
                        double greyscale_org = 0.2989 * p_org[0] + 0.5870 * p_org[1] + 0.1140 * p_org[2];
                        double dif = greyscale_tpl - greyscale_org;
                        aktSAD += dif*dif;
                    }
                }
            if (aktSAD < minSAD)
            {
                minSAD = aktSAD;
                _ref->found_x = xouter;
                _ref->found_y = youter;
            }
        }

//    ESP_LOGD(TAG, "FindTemplate 06");


    if (_ref->alignment_algo == 2)
        CalculateSimularities(rgb_template, _ref->found_x, _ref->found_y, ow, oh, min, avg, max, SAD, _ref->fastalg_SAD, _ref->fastalg_SAD_criteria);

//    ESP_LOGD(TAG, "FindTemplate 07");

    _ref->fastalg_x = _ref->found_x;
    _ref->fastalg_y = _ref->found_y;
    _ref->fastalg_min = min;
    _ref->fastalg_avg = avg;
    _ref->fastalg_max = max;
    _ref->fastalg_SAD = SAD;

    
/*#ifdef DEBUG_DETAIL_ON
    std::string zw = "\t" + _ref->image_file + "\tt1_x_y:\t" + std::to_string(_ref->fastalg_x) + "\t" + std::to_string(_ref->fastalg_y);
    zw = zw + "\tpara1_found_min_avg_max_SAD:\t" + std::to_string(min) + "\t" + std::to_string(avg) + "\t" + std::to_string(max) + "\t"+ std::to_string(SAD);
    LogFile.WriteToDedicatedFile("/sdcard/alignment.txt", zw);
#endif*/

    RGBImageRelease();
    stbi_image_free(rgb_template);
    
//    ESP_LOGD(TAG, "FindTemplate 08");

    return false;
}



bool CFindTemplate::CalculateSimularities(uint8_t* _rgb_tmpl, int _startx, int _starty, int _sizex, int _sizey, int &min, float &avg, int &max, float &SAD, float _SADold, float _SADcrit)
{
    double dif;
    double minDif = 65535;
    double maxDif = 0;
    double avgDifSum = 0;
    long int anz = 0;
    double aktSAD = 0;    

    int xouter, youter, _ch;

    for (xouter = 0; xouter <= _sizex; xouter++)
        for (youter = 0; youter <= _sizey; ++youter)
        {
            stbi_uc* p_org = rgb_image + (channels * ((youter + _starty) * width + (xouter + _startx)));
            stbi_uc* p_tpl = _rgb_tmpl + (channels * (youter * tpl_width + xouter));
            if (channels == 3)
            {
                double greyscale_tpl = 0.2989 * p_tpl[0] + 0.5870 * p_tpl[1] + 0.1140 * p_tpl[2];
                double greyscale_org = 0.2989 * p_org[0] + 0.5870 * p_org[1] + 0.1140 * p_org[2];
                dif = greyscale_tpl - greyscale_org;
                dif = dif*dif;
                aktSAD += dif;
                if (dif < minDif) minDif = dif;
                if (dif > maxDif) maxDif = dif;
                anz++;
            }
        }

    avg = aktSAD / anz;
    min = minDif;
    max = maxDif;
    SAD = sqrt(aktSAD) / anz;

    float _SADdif = abs(SAD - _SADold);

    ESP_LOGD(TAG, "Count %ld, aktSAD %fd, SAD_now: %fd, _SAD_old: %f, _SAD_dif:%f, _SAD_crit:%f", anz, aktSAD, SAD, _SADold, _SADdif, _SADcrit);

    if (_SADdif <= _SADcrit)
        return true;

    return false;
}
