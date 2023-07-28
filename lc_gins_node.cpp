/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-03-16 02:27:44
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2023-07-26 22:40:24
 * @FilePath: /ESKF-LCGINS/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <fstream>
#include <string.h>
#include "common.h"
#include "gi_lib.h"


#define DEBUG_MODE

#define BUFF_LEN 1024

// int decode_buff(const char* buff, const char sep, const int max_data_num, double* data)
// {
//     int len = strlen(buff);
//     int j = 0;
//     for (size_t i = 0; i < len; i++)
//     {
//         /* code */
//         if (buff[i] == sep || buff[i] == '\n')
//         {
//             sscanf(buff, "%lf", data + j);

//         }
//     }
    
// }

int main(int argc, char** argv) {
    const char* path_root = "/mnt/hgfs/home/code/LC_GINS/dataset/";
    char path_imu[128] = { 0 };
    char path_gnss[128] = { 0 };
    char path_result[128] = { 0 };
    char buff[BUFF_LEN] = { 0 };
    FILE *fpin_imu = NULL;
    FILE *fpin_gnss = NULL;
    FILE *fpout_result = NULL;
    double time_stamp = 0.0;
    status_t ret = GILIB_STATUS_OK;
    imu_t imu = { 0 };
    gnss_t gnss = { 0 };
    GILib gilib;
    navstate_t nav;
    strcpy(path_imu, path_root);
    strcat(path_imu, "Leador-A15.txt");
    strcpy(path_gnss, path_root);
    strcat(path_gnss, "GNSS-RTK.txt");
    strcpy(path_result, path_root);
    strcat(path_result, "result.txt");
    


    fpin_imu = fopen(path_imu, "r");
    fpin_gnss = fopen(path_gnss, "r");
    fpout_result = fopen(path_result, "w");
    if (fpin_imu == NULL || fpin_gnss == NULL || fpout_result == NULL)
    {
        printf("[open file error]: \n");
        return -1;
    }

    // 3. 循环
    while (!feof(fpin_imu))
    {
        memset(buff, 0, strlen(buff));
        fgets(buff, BUFF_LEN, fpin_imu);
        sscanf(buff, "%lf %lf %lf %lf %lf %lf %lf", 
            &imu.time, 
            &imu.gyro_deg(0), &imu.gyro_deg(1), &imu.gyro_deg(2), 
            &imu.acc(0), &imu.acc(1), &imu.acc(2));  
        // imu.gyro_deg = imu.gyro_deg / 0.005 * RAD2DEG;
        // imu.acc = imu.acc / 0.005;
        #ifdef DEBUG_MODE
            printf("imu: %.8lf  %.8lf  %.8lf\n", imu.time, imu.gyro_deg(2), imu.acc(2));
        #endif
        ret = gilib.process(prctype_imu, &imu, sizeof(imu_t), NULL, 0);
        ret = gilib.return_nav(nav);
        fprintf(fpout_result, "%.3lf", nav.time_stamp);
        fprintf(fpout_result, "   %.5lf %.5lf %.5lf", nav.blh_deg[0], nav.blh_deg[1], nav.blh_deg[2]);
        fprintf(fpout_result, "   %.5lf %.5lf %.5lf", nav.vel_ned[0], nav.vel_ned[1], nav.vel_ned[2]);
        fprintf(fpout_result, "\n");
        #ifdef DEBUG_MODE
        if (nav.flag)
        {
            printf("%.3lf", nav.time_stamp);
            printf("   %.5lf %.5lf %.5lf", nav.blh_deg[0], nav.blh_deg[1], nav.blh_deg[2]);
            printf("   %.5lf %.5lf %.5lf", nav.vel_ned[0], nav.vel_ned[1], nav.vel_ned[2]);
            printf("\n");
        }
        
        
        #endif
    }

    //
    fclose(fpin_imu);
    fclose(fpin_gnss);
    fclose(fpout_result);
    return 0;

}
