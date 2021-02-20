#include "lib1.h"

static  size_t                       arr_size = 1500;                                        // Default number of measurement points
const   size_t                       arr_offset = 50;                                        // Number of extra measurement points, which will be removed to avoid the intial jump in the signal
const   size_t                       dft_upper_freq = 110;                                   // DFT lowest frequency
const   size_t                       dft_lower_freq = 0;                                     // DFT lowest frequency
const   size_t                       dft_interval = dft_upper_freq - dft_lower_freq + 1;     // DFT total number of output frequencies
static  double                       t_measure = 0.0;                                        // Measurement duration (will be measured using system timer micros()
static  double                       t_set = 0.0;                                            // Set value of measurement duration
static  double                       val_min = 1.0E12;                                       // Minimum value of min-max measurement
static  double                       val_max = 0.0;                                          // Maximum value of min-max measurement
static  double                       factor_unit = 1.0;                                      // Unit conversion factor. Default unit is kA/m
static  unsigned int                 freq_main = 0;                                             // Fundamental frequency
static  std::vector<double>          dat_meas(arr_size);                                     // This vector stores measurement data
static  std::vector<double>          dat_dft(dft_interval);                                  // This vector stores DFT results (frequencies)
static  std::vector<double>          dat_comp(3);                                            // This vector stores values computed from measurement data (1) average (2) RMS (3) standard deviation
static  DeviceSettings               settings;     
static  enter_position               set;     


#define baud_rate 200000    //Baud Rate

//static  enter_position                     set;     

Genie Global::genie;

void setup()
{
  //WiFi.off();

    if ( EEPROM.read(0) != (uint8_t)1)
              {
                    initialize(settings);
                    EEPROM.put(0, (uint8_t)1);
                    EEPROM.put(1, settings);
                    
                } 
    else {
            EEPROM.get(1, settings);
      }
              
  setADCSampleTime(ADC_SampleTime_480Cycles); // possible values: 3, 15, 28, 56, 84, 112, 144, 480. Default: 480
  Serial1.begin(baud_rate);
  pinMode(AnalogSensorPin, INPUT);
    
   // pinMode(AnalogSensorPin, INPUT);  
   pinMode(menu_or_zero, INPUT_PULLUP); attachInterrupt(menu_or_zero, enterINTR, RISING);
   pinMode(down, INPUT_PULLUP);  attachInterrupt(down, downINTR,RISING);
   pinMode(up, INPUT_PULLUP);    attachInterrupt(up, upINTR,RISING);
   pinMode(right, INPUT_PULLUP);           attachInterrupt(right, rightINTR, RISING);
   pinMode(left, INPUT_PULLUP);           attachInterrupt(left, leftINTR, RISING);
      
      
  Global::genie.Begin(Serial1);           // Use Serial1 for talking to the Genie Library, and to the 4D Systems display 
  Particle.syncTime();            // sync to current time to the particle cloud
  Time.zone(+2);                  // add time zone of your country, Germany’s time zone is (+2)
	
	// Reset the Displays  (IMPORTANT: this can prevent out-of-sync issues, slow response etc)
  pinMode(DisplayRST, OUTPUT);
  digitalWrite(DisplayRST, 0);
  delay(100);
	
  digitalWrite(DisplayRST, 1);    // Set D4 on Photon to Output to control the reset line to Display
  delay (3500);                   // IMPORTANT: let the display start up after the reset

   Global::genie.WriteObject(GENIE_OBJ_FORM, 2, 1);
}

// Do random non-important code here
void loop() {
           //   down_button.Update();
          // if(down_button.clicks != 0) function1 = down_button.clicks;
             if (t_set > 1.0E-10)
                {
                    dat_meas.resize(arr_size + arr_offset);
                    data_read(dat_meas, t_measure, arr_offset, 1.0);
                    arr_size = arr_size * t_set / t_measure;
                    dat_meas.resize(arr_size + arr_offset);
                    data_read(dat_meas, t_measure, arr_offset, factor_unit*settings.calibration_factor);
            
                    if (settings.filters[0]) filter_apply(dat_meas, filter(5, &gaussian), 3);
                    if (settings.filters[1]) filter_apply(dat_meas, filter(21,&gaussian), 2);
                    if (settings.filters[2]) filter_apply(dat_meas, filter(51,&gaussian), 1);
                    if (settings.filters[3]) filter_apply(dat_meas, filter(7, &average), 3);
                    if (settings.filters[4]) filter_apply(dat_meas, filter(3, &gaussian), 3);
                    if (settings.filters[5]) exit(-1);
                   	 
                 if (settings.display_value[0] + settings.display_value[1] + settings.display_value[2])
                              {
                                   dat_comp = data_analyze(dat_meas, settings.zero_point);
                              }
                    
                    if (settings.adjustment_value > 1.0E-10)
                    {
                        switch (settings.measurement_mode)
                        {
                            case ST1:
                                settings.calibration_factor *= settings.adjustment_value / dat_comp[0];
                                break;
                            case ST2:
                                settings.calibration_factor *= settings.adjustment_value / dat_comp[1];
                                break;
                            default:
                                exit(-1);
                        }
                        if (is_measurement_good(dat_meas)) settings.adjustment_value = 0.0;
                    }
                    
                    if (settings.display_value[3]) dft_compute(dat_meas, dat_dft, dft_lower_freq, t_measure);
                    
                    if (settings.display_value[0])
                    {
                        val_min = std::min(val_min, dat_comp[0]);
                        val_max = std::max(val_max, dat_comp[0]);
                    } else {
                        val_min = 1.0E12;
                        val_max = 0.0;
                    }
                    
                    if (settings.display_value[1])
                    {
                        val_min = std::min(val_min, dat_comp[1]);
                        val_max = std::max(val_max, dat_comp[1]);
                    } else {
                        val_min = 1.0E12;
                        val_max = 0.0;
                    }
                    
                    if (settings.display_value[3])
                    {
                        dft_compute(dat_meas, dat_dft, dft_lower_freq, t_measure);
                        freq_main = std::max_element(dat_dft.begin(), dat_dft.end()) - dat_dft.begin() + dft_lower_freq;
                    }
                }
            Global::genie.WriteStr(5,dat_comp[0]);
                        Global::genie.WriteStr(4,UST3);
            delay(300);
         
}






