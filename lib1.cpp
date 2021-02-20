#include "lib1.h"

enter_position       set; 


std::vector<double> filter(const size_t &filter_support, double (*f)(const double&, const size_t&))
{
    if ( filter_support % 2 == 0)
    {
        Serial.println("Wrong vector length! Odd number expected.");
        delay(3000);
        exit(-1);
    }
    std::vector<double> out(filter_support);
    const size_t n = filter_support / 2;
    double sum = 0.0;
    for (int i = filter_support-1; i--;)
    {
        out[i] = f(i-n, filter_support);
        sum += out[i];
    }
    for (int i = filter_support-1; i--;)
    {
        out[i] /= sum;
    }
    return out;
}

//-----------------------------------------------------------------------------------------------------

double gaussian(const double &x, const size_t &filter_support)
{
    if ( filter_support % 2 == 0)
    {
        Serial.println("Wrong vector length! Odd number expected.");
        //throw std::invalid_argument( "FEHLER: gerade Zahl als Filtergroesse!" );
        delay(3000);
        exit(-1);
    }
    double sigma = (filter_support / 2) / 3.0;
    return exp(- x*x / (2.0*sigma*sigma)) / sigma / sqrt(2.0*PI);
}



double average(const double &x, const size_t &filter_support)
{
    return 1.0 / (double)filter_support;
}

//-----------------------------------------------------------------------------------------------------

void filter_apply(std::vector<double> &data, const std::vector<double> &filter, const size_t &n_filtering)
{
    const size_t filter_support = filter.size();
    const size_t hfg = filter_support / 2; // Hälfte der Filtergröße
    const size_t arr_size = data.size();
    static std::vector<double> temp(arr_size);
    double faktor = 0.0;
    
    for (int k = 0; k < n_filtering; ++k)
    {
        temp = data;
        for (int i = 0; i < arr_size; ++i)
        {
            data[i] = 0.0;
            for (int j = 0; j < filter_support; ++j)
            {
                data[i] += filter[j] * temp[min(max(0,i-hfg+j), arr_size-1)];
            }
        }
    }
}

//-----------------------------------------------------------------------------------------------------

void data_read(std::vector<double> &data, double &t_measure, const size_t &offset, const double &correction_factor)
{
    unsigned long t_1, t_2;
    size_t arr_size = data.size();
    ATOMIC_BLOCK()
    {
        t_1 = micros();
        for(int i = arr_size-1; i--;)
        {
            data[i] = analogRead(A0);
        }
        t_2 = micros();
    }
    if (offset > 0) data.resize(arr_size - offset);
    std::reverse(data.begin(), data.end());
    t_measure = (t_2 - t_1) * 1.0E-6 * arr_size / (arr_size+offset);
    arr_size = data.size();
    if (std::abs(correction_factor - 1.0) > 1.0E-12)
    {
        for(int i = arr_size-1; i--;)
        {
            data[i] *= correction_factor;
          
        }
    }
}

//-----------------------------------------------------------------------------------------------------

std::vector<double> data_analyze(const std::vector<double> &data, const std::vector<double> &zero_point)
{
    std::vector<double> out(3, 0.0);
    const size_t n_data = data.size();
    for (int i = n_data-1; i--;)
    {
        out[0] += data[i];
        out[1] += data[i] * data[i];
    }
    out[0] /= n_data;
    out[1] /= n_data;
    out[1] = std::sqrt(out[1]);
    for (int i = n_data-1; i--;)
    {
        out[2] += (data[i] - out[0]) * (data[i] - out[0]);
        
    }
    out[2] /= n_data - 1;

    out[2] = std::sqrt(out[2]);

    out[0] -= - zero_point[0];

    out[1] -= - zero_point[1];

    out[2] -= - zero_point[2];

    return out;
}

//-----------------------------------------------------------------------------------------------------

bool is_measurement_good(const std::vector<double> &data)
{
    bool is_good;
    double avg = 0.0;
    double avg_2 = 0.0;
    double stdev = 0.0;
    unsigned int cnt_2 = 0;
    const size_t n_data = data.size();
    for (int i = n_data-1; i--;)
    {
        avg += data[i];
        if (i < n_data)
        {
            avg_2 += data[i];
            cnt_2++;
        }
    }
    avg /= n_data;
    avg_2 /= cnt_2;
    
    for (int i = n_data-1; i--;)
    {
        stdev += (data[i] - avg) * (data[i] - avg);
    }
    stdev /= n_data - 1;
    stdev = std::sqrt(stdev);
    return ( std::abs(avg - avg_2) > stdev ) ? false : true;
}

//-----------------------------------------------------------------------------------------------------

void data_print(const std::vector<double> &data, const size_t &offset, const unsigned int &t_delay)
{
    size_t arr_size = data.size();
    for(int i = offset; i < arr_size - offset; ++i) {
        Serial.println(data[i]);
    }
    delay(t_delay);
}


//-----------------------------------------------------------------------------------------------------

void dft_compute(const std::vector<double> &dat_in, std::vector<double> &dat_out, const size_t &dft_lower_freq, const double &t_measure)
{
	double angle, real, imag;
	const size_t n_input = dat_in.size();
	const size_t n_output = dat_out.size();
	for (size_t k = 0; k < n_output; k++) {
		real = 0.0;
		imag = 0.0;
		for (size_t t = 0; t < n_input; t++) {
			angle = 2 * PI * t * (dft_lower_freq + k) * t_measure / n_input;
			//dat_out[k] +=  dat_in[t] * std::cos(angle) + inimag[t] * std::sin(angle);
			//outimag[k] += -dat_in[t] * std::sin(angle) + inimag[t] * std::cos(angle);
			real +=  dat_in[t] * (t_measure / n_input) * std::cos(angle);
			imag += -dat_in[t] * (t_measure / n_input) * std::sin(angle);
		}
		dat_out[k] = std::sqrt(real*real + imag*imag);
	}
}

// -----------------------------------------------------------------------------------------------------

void initialize(DeviceSettings &settings)
{
    settings.calibration_factor = 1.0;
    settings.measurement_mode = UST0;
    settings.integration_time = 2;
    settings.unit = UST0;
    settings.screen = SST0;
    settings.adjustment_value = 0.0;
    settings.cancel = false;
    settings.filters = {ST0, ST0, ST0, ST0, ST0, ST0};
    settings.display_value = {ST1, ST0, ST0, ST0, ST0, ST0};
    settings.zero_point = {0.0, 0.0, 0.0};
}

//-----------------------------------------------------------------------------------------------------

unsigned long waitPeriod = millis();

// Button press interrupt service routine
void enterINTR(){   
  
            if (millis() >= waitPeriod )   // debounce time = 50milliseconds
             {
                // buttonPressFlag = true;
                set.enter++;
                waitPeriod = millis()+ 50;
            }
                       if(set.enter==1)
                       {
                           Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,6,0);
                           Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,5,0);
                           Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,3,0);
                           Global::genie.WriteObject(GENIE_OBJ_FORM, 3, 1);
                           set.down_bool=false;
                       }
             else if( set.enter==2)
             {
                Global::genie.WriteObject(GENIE_OBJ_FORM, 2, 1);
                
            }
           if(set.enter==2)
             {
                 set.enter=0;
             }

}

void downINTR(){   
                 if( set.down_bool==false){// && set.switch_state==false){
                     if (millis() >= waitPeriod  && set.switch_state==false)
                         {
                                set.down_up++;
                                if(set.down_up>2) set.down_up=2;
                                switch(set.down_up)
                                      {
                                          case 0:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,2,0);
                                                set.right=0;
                                                break;
                                                
                                          case 1:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,2,1);
                                               set.right=1;
                                                break;
                                                
                                          case 2:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,2,2);
                                                    set.right=2;
                                                break;
                                      }
                                      set.switch_state=true;
                                         waitPeriod = millis()+ 50;
                         }
                          set.switch_state=false;
                         
                 }
           
               if(set.down_bool==true ){//&& set.switch_state==false){
                       if (millis() >= waitPeriod  && set.switch_state==false)
                     {
                            set.down++;
                            if(set.down>=4) set.down=4;
                            if(set.set_num==3){if(set.down>=2) set.down=2;}
                            switch(set.down)
                                  {
                                      case 1:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,2);
                                             break;
                                           
                                      case 2://if(set.set_num==3) set.down=2;
                                             Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,3);
                                             break;
                                             
                                      case 3:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,4);
                                             break;
                                             
                                      case 4:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,5);
                                             break;
                            } 
                            set.switch_state=true;
                            waitPeriod = millis()+ 50;
                     }
                    set.switch_state=false;
           }
            Global::genie.WriteStr(12,set.down);
            Global::genie.WriteStr(11, set.down_up);
} 

void upINTR(){   
                  if( set.down_bool==false){// && set.switch_state==false){
                           if (millis() >= waitPeriod  && set.switch_state==false)
                         {
                             set.down_up--;
                             if(set.down_up<=0)set.down_up =0;
                            switch(set.down_up)
                                  {
                                      case 0:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,2,0);
                                            set.right=0;
                                              
                                            break;
                                            
                                      case 1:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,2,1);
                                             set.right=1;
                                             break;
                                            
                                      case 2:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,2,2);
                                            set.right=2;
                                            break;
                                  }
                                  set.switch_state=true;
                                  waitPeriod = millis()+ 50;
                         }
                         set.switch_state=false;
                   }
                if( set.down_bool==true ){//&& set.switch_state==false){
                           if (millis() >= waitPeriod  && set.switch_state==false)
                         {  
                                set.down--;
                                if(set.down <=1) set.down =1;
                              //  if(set.set_num==3) set.down=1;
                                switch(set.down)
                                      {
                                          case 1:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,2);
                                            break;
                                            
                                          case 2:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,3);
                                                 break;
                                               
                                          case 3:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,4);
                                                 break;
                                                 
                                          case 4:
                                                 Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,set.set_num,5);
                                                 break;
                                } 
                                set.switch_state=true;
                                waitPeriod = millis()+ 50;
                         }
                         set.switch_state=false;   
               }
   
            Global::genie.WriteStr(12,set.down);
            Global::genie.WriteStr(11, set.down_up);
} 


void rightINTR(){   
    set.down_boolean = true;

            if (millis() >= waitPeriod )
          {
              switch(set.right)
              {
                  case 0:
                              set.down_bool=true;
                              set.down=0;
                              set.set_num=6;
                              Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,6,1);
                              break;
                        
                        case 1:
                              set.set_num=5;
                              set.down=0;
                              set.down_bool=true;
                              Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,5,1);
                              break;
                        
                        case 2:
                              set.set_num=3;
                              set.down=0;
                              set.down_bool=true;
                              Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,3,1);
                              break;
              }
                 waitPeriod = millis()+ 50;
          }
}

void leftINTR(){   
    if (millis() >= waitPeriod )
  {
        switch(set.left)
              {
                 case 0:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,6,0);
                         break;
                         
                 case 1:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,5,0);
                        break;
                 
                 case 2:Global::genie.WriteObject(GENIE_OBJ_USERIMAGES,3,0);
                       break;
                         
              }
                set.down_bool=false;
                set.down=0;
                waitPeriod = millis()+ 50;
  }
}

void set_position(enter_position &set)
{
    set.enter = 0; 
    set.down=0;
    set.up=0;
    set.left;
    set.right=0;
    set.down_up=0;
    set.sub_time=0;
    set.sub_unit=0;
    set.sub_graphics=0;
    set.val_stop=0;
    set.left=set.down_up;
    set.down_bool=false;
    set.set_num;
    set.down_boolean=false;
    set.count_down=0;
    set.switch_state=false;
}
   
   
   
   