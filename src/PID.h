#include <algorithm>
#include "main.h"
class PID {
	public:

        PID();
        PID(double p, double i, double d);
        PID(double p, double i, double d, double min, double max, double i_min, double i_max, double i_offset);

        void compute(double target, double input);
        double getOutput();
        void setConstants(double p, double i, double d);
        void setBounds(double min, double max);
        void setIBounds(double i_min, double i_max);

	private:
        bool initialized;
        double p_val;
        double i_val;
        double d_val;
        double _min;
        double _max;
        double _i_min;
        double _i_max;
        double i_term;
        uint32_t lasttime;
        double lastinput;
        double output;
};
