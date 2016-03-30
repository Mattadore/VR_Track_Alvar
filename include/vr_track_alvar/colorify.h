//makes the AR tags prettier.
//which I consider a solid investment of time.
#include <std_msgs/ColorRGBA.h>
#include <math.h>
namespace alvar {
    std_msgs::ColorRGBA colormarker(int markID) {
        std_msgs::ColorRGBA out;
        out.a = 1.0;
        //change maxID if you want a different scale, but it'll make there be less difference among close colors
        int maxID = 18;
        int scale = 7; //a rotational factor that ensures adjacent colors are not adjacent colors but all colors are still unique
        //maxID and scale
        markID *= scale;
        markID %= maxID;
        double v = 0.9;
        double s = 1.0; //vary these if we need more colors
        double h = ((double)markID) / ((double) maxID);
        double hh, p, q, t, ff;
        long i;
        struct {
            double h;
            double s;
            double v;
        } in;
        in.h=h;
        in.s=s;
        in.v=v;
        if(in.s <= 0.0) {    
            out.r = in.v;
            out.g = in.v;
            out.b = in.v;
            return out;
        }
        hh = in.h;
        hh *= 6.0;
        i = (long)hh;
        ff = hh - i;
        p = in.v * (1.0 - in.s);
        q = in.v * (1.0 - (in.s * ff));
        t = in.v * (1.0 - (in.s * (1.0 - ff)));
        switch(i) {
        case 0:
            out.r = in.v;
            out.g = t;
            out.b = p;
            break;
        case 1:
            out.r = q;
            out.g = in.v;
            out.b = p;
            break;
        case 2:
            out.r = p;
            out.g = in.v;
            out.b = t;
            break;

        case 3:
            out.r = p;
            out.g = q;
            out.b = in.v;
            break;
        case 4:
            out.r = t;
            out.g = p;
            out.b = in.v;
            break;
        case 5:
        default:
            out.r = in.v;
            out.g = p;
            out.b = q;
            break;
        }
    }
}