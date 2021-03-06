#ifndef VEX_U_2018_V5_PID_H
#define VEX_U_2018_V5_PID_H

#include "afr-vexu-lib/action.h"

namespace AFR::VexU::BaseAction{
    template<typename Read_T, typename Write_T>
    class pid_action : public action{
        double _p_value;
        double _i_value;
        double _d_value;
        Write_T _min_value;
        Write_T _max_value;
        Write_T _min_i_value;
        Write_T _max_i_value;
        Write_T _offset;
        Read_T* const _value_pointer;
        Read_T _setpoint;
        
        double last_error;
        Read_T last_value;
        Write_T i_term;
        bool running;
     
        error_t update_private(const double& delta_seconds) override;
        
        /**
         * Sets PID constants
         * @param p_value P constant
         * @param i_value I constant
         * @param d_value D constant
         * @return error_t value if error encountered
         */        
        error_t set_pid_constants(double& p_value, double& i_value, double& d_value);
        
        /**
         * Sets output bounds
         * @param min_value minimum allowable value
         * @param max_value maximum allowable value
         * @return error_t value if error encountered
         */                
        error_t set_bounds(Write_T& min_value, Write_T& max_value);
        
        /**
         * Sets I term bounds
         * @param min_i_value minimun allowable I term
         * @param max_i_value maximum allowable I term
         * @return error_t value if error encountered
         */                
        error_t set_i_bounds(Write_T& min_i_value, Write_T& max_i_value);
        
        /**
         * Sets output offset
         * @param offset offset for calculated output
         * @return error_t value if error encountered
         */        
        error_t set_offset(Write_T& offset);
        
        /**
         * Sets target for controller
         * @param setpoint target value for controller
         * @return error_t value if error encountered
         */                
        error_t set_target(Read_T& setpoint);
        
        public:
        /**
         * Creates a PID action
         * @param update_period passed to scheduled
         * @param commandable passed to action
         * @param p_value P constant
         * @param i_value I constant
         * @param d_value D constant
         * @param min_value minimum allowable value
         * @param max_value maximum allowable value
         * @param min_i_value minimun allowable I term
         * @param max_i_value maximum allowable I term
         * @param offset offset for calculated output
         * @param value_ptr the pointer to the value to be copied every update
         * @param setpoint target value for controller
         * @param result error_t value if error encountered
         */            
            pid_action(const scheduled_update_t& update_period, commandable& commandable, double& p_value,
                            double& i_value, double& d_value, Write_T& min_value, Write_T& max_value, 
                            Write_T& min_i_value, Write_T& max_i_value, Write_T& offset, Read_T* const value_pointer, 
                            Read_T& setpoint, error_t* result = nullptr);                       
    };
    template<typename Read_T, typename Write_T>
    error_t pid_action<Read_T, Write_T>::update_private(const double& delta_seconds){
        double error = static_cast<double>(_setpoint - *_value_pointer);
        Write_T p_term = static_cast<Write_T>(_p_value*error);
        
        Write_T d_term;
        
        //Only calculate i and d terms if reasonable time delta and enabled
        if(running && delta_seconds > 0.001) {
            i_term += static_cast<Write_T>(_i_term*error*delta_seconds);
            
            //clamp i value
            if(i_term > _max_i_value) {
                i_term = _max_i_value;
            }
            else if(i_term < _min_i_value) {
                i_term = _min_i_value;
            }
            
            Write_T d_term = static_cast<Write_T>(static_cast<double>(last_value-*_value_pointer)*_d_value/delta_seconds);
        }
        else {
            d_term = 0;
            running = true;
        }
        Write_T write_value = p_term + i_term + d_term + _offset;
        
        //clamp write value
        if(write_value > _max_value) {
            write_value = _max_value;
        }
        else if(write_value < _min_value) {
            write_value = _min_value;
        }
        
        last_value = *_value_pointer;
        return commandable_.set_value(write_value);
    }
    
    template<typename Read_T, typename Write_T>
    error_t pid_action<Read_T, Write_T>::set_pid_constants(double& p_value, double& i_value, double& d_value) {
        _p_value = p_value;
        _i_value = i_value;
        _d_value = d_value;
        return SUCCESS;
    }
    
    template<typename Read_T, typename Write_T>
    error_t pid_action<Read_T, Write_T>::set_bounds(Write_T& min_value, Write_T& max_value) {
        _min_value = min_value;
        _max_value = max_value;
        return SUCCESS;
    }
    
    template<typename Read_T, typename Write_T>
    error_t pid_action<Read_T, Write_T>::set_i_bounds(Write_T& min_i_value, Write_T& max_i_value) {
        _min_i_value = min_i_value;
        _max_i_value = max_i_value;
        return SUCCESS;
    }
    
    template<typename Read_T, typename Write_T>
    error_t pid_action<Read_T, Write_T>::set_offset(Write_T& offset) {
        _offset = offset;
        return SUCCESS;
    }
    
    template<typename Read_T, typename Write_T>
    error_t pid_action<Read_T, Write_T>::set_target(Read_T& setpoint) {
        _setpoint = setpoint;
        return SUCCESS;
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-non-const-parameter"
    template<typename Read_T, typename Write_T>
    pid_action(const scheduled_update_t& update_period, commandable& commandable, double& p_value,
                            double& i_value, double& d_value, Write_T& min_value, Write_T& max_value, 
                            Write_T& min_i_value, Write_T& max_i_value, Write_T& offset, Read_T* const value_pointer, 
                            Read_T& setpoint, error_t* result = nullptr) : 
                                          action(update_period, commandable, result),
                                          _p_value(p_value),
                                          _i_value(i_value),
                                          _d_value(d_value),
                                          _min_value(min_value),
                                          _max_value(max_value),
                                          _min_i_value(min_i_value),
                                          _max_i_value(max_i_value),
                                          _offset(offset),
                                          _value_pointer(value_pointer),
                                          _setpoint(setpoint),
                                          last_error(0),
                                          last_value(0),
                                          i_term(0),
                                          running(false){}

#pragma clang diagnostic pop
      
}

#endif