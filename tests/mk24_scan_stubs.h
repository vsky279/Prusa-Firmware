// Hardware boundary doubles. The probing/retry control flow is firmware source.
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#define NEW_XYZCAL
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define MESH_HOME_Z_SEARCH 5.f
#define FIND_BED_INDUCTION_SENSOR_POINT_Z_STEP 0.1f
#define EEPROM_BED_CALIBRATION_CENTER 0
#define EEPROM_BED_CALIBRATION_VEC_X 0
#define EEPROM_BED_CALIBRATION_VEC_Y 0
#define MANUAL_Y_HOME_POS -2.2f
#define MESH_MEAS_NUM_X_POINTS 3
float BED_X(int i) { return 1.f+34.f*i; }
float BED_Y(int i) { return 1.f+34.f*i; }
#define SERIAL_ECHOPGM(...) ((void)0)
static const float bed_ref_points_4[] = {12,4,220,4,220,196,12,196};
struct MeshStub { float z_values[7][7]; } mbl;
static float current_position[4], homing_feedrate[4]={3000,3000,800,0};
static std::vector<int> visits;
static int fail_mode=0;
void refresh_cmd_timeout() {}
void go_to_current(float) {
    if (fabs(current_position[0]-220.f)<0.01f && fabs(current_position[1]-196.f)<0.01f) {
        fprintf(stderr,"Movement to defective XYZ reference\n"); exit(4);
    }
}
void world2machine_update(float*,float*,float*) {}
void world2machine_update_current() {}
void eeprom_update_block_notify(void*,void*,int) { ++writes; }
BedSkewOffsetDetectionResultType find_bed_induction_sensor_point_xy(int8_t) {
    int point=-1;
    for (int i=0;i<4;++i)
        if (fabs(current_position[0]-bed_ref_points_4[2*i])<0.01f &&
            fabs(current_position[1]-bed_ref_points_4[2*i+1])<0.01f) point=i;
    if (point<0 || point==2) { fprintf(stderr,"Unexpected physical probe %d\n",point); exit(1); }
    visits.push_back(point);
    if (fail_mode==1) return BED_SKEW_OFFSET_DETECTION_POINT_NOT_FOUND;
    if (fail_mode==2 || (fail_mode==3 && visits.size()==1)) return BED_SKEW_OFFSET_DETECTION_POINT_SCAN_FAILED;
    return BED_SKEW_OFFSET_DETECTION_PERFECT;
}
static void scan_tests() {
    for (int mode=0;mode<4;++mode) {
        fail_mode=mode; visits.clear(); writes=0;
        uint8_t mask=0;
        extern BedSkewOffsetDetectionResultType find_bed_offset_and_skew(int8_t,uint8_t&);
        int result=find_bed_offset_and_skew(20,mask);
        if (mode==0 || mode==3) {
            if(result<0 || visits!=std::vector<int>(mode==0 ? std::initializer_list<int>{0,1,3} : std::initializer_list<int>{0,1,3,0,1,3})) exit(2);
        } else {
            if(result>=0 || writes!=0 || visits.size()!=(mode==1 ? 1u : 6u)) exit(3);
        }
    }
    puts("Actual XYZ scan: success, detection failure, retry success, retry exhaustion passed");
}
