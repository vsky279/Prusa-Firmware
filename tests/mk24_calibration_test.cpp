// Included after the actual firmware fitting function by run_mk24_calibration.py.
#include <cassert>
#include <cstdio>
#include <limits>
#include <cstdlib>
#undef assert
#define assert(condition) do { if (!(condition)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while(0)

static const float refs[] = {12,4,220,4,220,196,12,196};
static int cases = 0;
static void trial(float rotation, float skew, float tx, float ty, float noise) {
    float p[8], x[2], y[2], c[2];
    float a = rotation, b = rotation + skew;
    // Firmware fits machine->world unit-axis model, then returns its inverse.
    float det = cos(a)*cos(b)+sin(a)*sin(b);
    for (int i=0; i<4; ++i) {
        float u=refs[2*i]-tx, v=refs[2*i+1]-ty;
        p[2*i]=(cos(b)*u+sin(b)*v)/det + noise*(i%2 ? 1 : -1);
        p[2*i+1]=(-sin(a)*u+cos(a)*v)/det + noise*(i<2 ? 1 : -1);
    }
    p[4]=p[5]=std::numeric_limits<float>::quiet_NaN();
    int result=calculate_machine_skew_and_offset_LS(p,4,refs,x,y,c,0);
    if (result<0) fprintf(stderr,"Rejected trial rotation=%g skew=%g translation=%g noise=%g\n", rotation,skew,tx,noise);
    assert(result >= 0);
    for (int i : {0,1,3}) {
        float dx=x[0]*refs[2*i]+y[0]*refs[2*i+1]+c[0]-p[2*i];
        float dy=x[1]*refs[2*i]+y[1]*refs[2*i+1]+c[1]-p[2*i+1];
        assert(hypot(dx,dy) < 0.35f);
    }
    assert(isfinite(c[0]) && isfinite(c[1]));
    ++cases;
}
static void invalid(float *p) {
    float x[2], y[2], c[2];
    writes=0;
    assert(calculate_machine_skew_and_offset_LS(p,4,refs,x,y,c,0)<0);
    assert(writes==0);
    ++cases;
}
static void compare_old() {
    float p[8]={12,4,220,4,NAN,NAN,12,196};
    old_reconstruction(p);
    assert(!isfinite(p[5])); // exact horizontal front edge: division by zero
    puts("Original reconstruction: horizontal geometry produces non-finite Y (new fitter accepts measured triangle)");
    // Identical non-singular, skewed inputs for both actual fitters.
    float a=0.001f,b=0.004f,det=cos(a)*cos(b)+sin(a)*sin(b);
    float measured[8];
    for(int i=0;i<4;++i) {
        measured[2*i]=(cos(b)*refs[2*i]+sin(b)*refs[2*i+1])/det;
        measured[2*i+1]=(-sin(a)*refs[2*i]+cos(a)*refs[2*i+1])/det;
    }
    float old[8]; memcpy(old,measured,sizeof(old)); old_reconstruction(old);
    float ox[2],oy[2],oc[2],nx[2],ny[2],nc[2];
    assert(old_fitter(old,4,refs,ox,oy,oc,0)>=0);
    measured[4]=measured[5]=NAN;
    assert(calculate_machine_skew_and_offset_LS(measured,4,refs,nx,ny,nc,0)>=0);
    float old_error=0,new_error=0;
    for(int i : {0,1,3}) {
        old_error+=hypot(ox[0]*refs[2*i]+oy[0]*refs[2*i+1]+oc[0]-measured[2*i],ox[1]*refs[2*i]+oy[1]*refs[2*i+1]+oc[1]-measured[2*i+1]);
        new_error+=hypot(nx[0]*refs[2*i]+ny[0]*refs[2*i+1]+nc[0]-measured[2*i],nx[1]*refs[2*i]+ny[1]*refs[2*i+1]+nc[1]-measured[2*i+1]);
    }
    assert(new_error<old_error && new_error<0.001f);
    printf("Identical skewed inputs, sum measured-point errors: old=%g mm, new=%g mm\n",old_error,new_error);
}
static void boundaries() {
    for (int k=0;k<6;++k) {
        float a=0.f,b=(k<2 ? bed_skew_angle_mild : bed_skew_angle_extreme)*(k%2 ? 1.01f : 0.99f);
        if(k>=4) a=b; // pure rotation instead of skew
        float p[8],x[2],y[2],c[2],d=cos(a)*cos(b)+sin(a)*sin(b);
        for(int i=0;i<4;++i) {
            p[2*i]=(cos(b)*refs[2*i]+sin(b)*refs[2*i+1])/d;
            p[2*i+1]=(-sin(a)*refs[2*i]+cos(a)*refs[2*i+1])/d;
        }
        p[4]=p[5]=NAN; writes=0;
        int result=calculate_machine_skew_and_offset_LS(p,4,refs,x,y,c,0);
        assert(result==(k==0 || k==4 ? 0 : (k==3 || k==5 ? -2 : 1)));
        assert(writes==(result<0 ? 0 : 1));
        ++cases;
    }
    float p[8]={12,4,220,4,NAN,NAN,12,196},x[2],y[2],c[2];
    assert(calculate_machine_skew_and_offset_LS(p,3,refs,x,y,c,0)<0);
    ++cases;
}
int main() {
    for (float a : {-0.001f,0.f,0.001f})
        for (float b : {-0.0025f,0.f,0.0025f})
            for (float t : {-3.f,0.f,3.f})
                for (float noise : {0.f,0.02f,0.08f}) trial(a,b,t,-t,noise);
    float p[8]={12,4,220,4,NAN,NAN,12,196};
    p[0]=NAN; invalid(p); p[0]=INFINITY; invalid(p); p[0]=12;
    p[6]=220; p[7]=4; invalid(p); // repeated point
    p[6]=116; invalid(p); // collinear
    p[6]=12; p[7]=-196; invalid(p); // mirrored
    p[7]=4.00001f; invalid(p); // nearly collinear
    p[7]=196; p[2]=240; invalid(p); // inconsistent length/residual
    p[2]=220; p[0]=1.e30f; invalid(p); // overflow
    p[0]=12; p[6]=30; p[7]=196; invalid(p); // excessive skew must fail fit
    boundaries();
    printf("MK24 actual firmware fitter: %d cases passed\n",cases);
    scan_tests();
    compare_old();
}
