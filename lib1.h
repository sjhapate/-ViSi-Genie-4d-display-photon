
// check that the library is not connected yet
#ifndef Button_h // if the Button library is not connected
#define Button_h // then connect it

#define PI 3.14159265358979323846
#define PI 3.14159265358979323846

#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string.h>
#include <application.h>
#include <genieParticle.h>

const unsigned short ST0 = 0;
const unsigned short ST1 = 1;
const unsigned short ST2 = 2;
const unsigned short ST3 = 4;
const unsigned short ST4 = 8;
const unsigned short ST5 = 16;
const unsigned short ST6 = 32;


const unsigned short SST0 = 2; //screen switch
const unsigned short SST1 = 1; //screen switch

const unsigned short UST0 = 0;
const unsigned short UST1 = 1;
const unsigned short UST2 = 2;
const unsigned short UST3 = 3;
const unsigned short UST4 = 4;
const unsigned short UST5 = 5;

// the Button
//************************************** Analog and Digital pins declaration********************************************
const int AnalogSensorPin = A0;     // initialize Analog pin 0 as a INPUT for the reading of magnetic analog sensor value  
const int down = D2;          // initialize digital pin 2 as a unit Switch for different Calibration factors 
const int right = D3;       // initialize digital pin 3 as a different Intergation Time Switch for different integration time 
const int DisplayRST= D4;           // initialize digital pin 4 for the Reset Display
//const int RefPin = D5;              // initialize digital pin 5 as a INPUT Switch for Referance point (Zero point) 
const int left= D5;           // initialize digital pin 5 as a INPUT Switch for graphiccs(DFT or waveform) 
const int menu_or_zero = D6;         // initialize digital pin 6 as a INPUT Switch for dispaying AC and DC FORM 
const int up = D1;


// This #include statement was automatically added by the Particle IDE.


class Global
{

public:
    static Genie genie;

};

struct DeviceSettings {
    double calibration_factor;                                                      // factor with which raw sensor data has to be multiplied to obtain display value (unit conversion exluded!)
    unsigned short measurement_mode;                                                // ST0: null state / ST1: measuring in DC / ST2: measuring in AC
    unsigned short integration_time;                                                // ST1: 33.3 ms / ST2: 40.0 ms / ST3: 333 ms / ST4: 400 ms
    unsigned short unit;                                                            // ST1: kA/m (default) / ST2: A/cm / ST3: mT / ST4: G
    unsigned short screen;                                                          // ST1: field measurement / ST2: frequency measurement / ST3: waveform / ST4: DFT
    double adjustment_value;                                                        // nominal measurement value given by user
    bool cancel;                                                                    // set true for jumping out of dead loops
    std::vector<unsigned short> filters= std::vector<unsigned short>(6);            // ST1: 3 x Gauss filter (support 5 points) / ST2: 2 x Gauss filter (support 21 points)
                                                                                    // ST3: 1 x Gauss filter (support 51 points) / ST4: 3 x average filter (support 7) / ST5: 3 x Gauss filter (support 3 points)
    std::vector<unsigned short> display_value = std::vector<unsigned short>(6);     // ST1: average / ST2: standard deviation / ST3: RMS / ST4: min-max / ST5: fundamental frequency (first harmonic frequency using DFT)
    std::vector<double> zero_point = std::vector<double>(3);                        // setpoint by zero adjustment
};

struct enter_position{
            short enter; 
            short down;
            short up;
            short right;
            short left;
            short down_up;
            short sub_time;
            short sub_unit;
            short sub_graphics;
            short val_stop;
            short down_bool;
            short index_1;
            short index_2;
            short index_3;
            short index_4;
            short set_num;
            bool down_boolean=false;
            short count_down;
            bool switch_state;
};

void enterINTR();
void upINTR();
void downINTR();
void rightINTR();
void leftINTR();
int buttonHandler(); 

// apply the given filter to the data set for a given number of repetitions
void                filter_apply(std::vector<double> &data, const std::vector<double> &filter, const size_t &n_filtering);

// discrete Gaussian function with given number of support points
double              gaussian(const double &x, const size_t &filter_support);


int                 add_ex(const size_t &d,const size_t &c,const double &correction_factor);

// discrete rectangular function (for average filter) with given number of support points
double              average(const double &x, const size_t &filter_support);

// general discrete filter for given filter function and number of support points
std::vector<double> filter(const size_t &filter_support, double (*f)(const double&, const size_t&));

// initialize device settings with default values (will be applied only if no settings have been saved on EEPROM)
void                initialize(DeviceSettings &settings);

// read sensor data into given vector and return measurement time
void                data_read(std::vector<double> &data, double &t_measure, const size_t &offset, const double &correction_factor);

// output data using Serial.print()
void                data_print(const std::vector<double> &data, const size_t &offset, const unsigned int &t_delay);

// compute Discrete Fourier Transform limited within give frequency interval
void                dft_compute(const std::vector<double> &dat_in, std::vector<double> &dat_out, const size_t &dft_lower_freq, const double &t_measure);

// compute and return (1) average (2) RMS (3) standard deviation
std::vector<double> data_analyze(const std::vector<double> &data, const std::vector<double> &zero_point);

// check if the signal has changed significantly during data acquisition
bool                is_measurement_good(const std::vector<double> &data);

#endif