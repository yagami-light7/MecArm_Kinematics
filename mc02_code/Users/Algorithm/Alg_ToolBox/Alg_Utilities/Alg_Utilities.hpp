#ifndef ENGINEERING_ALG_UTILITIES_HPP
#define ENGINEERING_ALG_UTILITIES_HPP

#include "arm_math.h"

#ifdef __cplusplus
namespace Math
{
    // numerical calculation
    namespace num
    {
        /**
          * @brief          abs function
          * @param[in]      val
          * @retval         return abs(val)
          */
        template <typename T>
        T abs(T val)
        {
            return val > 0 ? val : -val;
        }

        /**
          * @brief          RSR(reciprocal square root) function 平方根倒数
          * @param[in]      val
          * @retval         return rsr(val)
          */
        template <typename T>
        float rsr(T val)
        {
            float halfnum = 0.5f * val;
            float y = val;
            long i = *(long *)&y;
            i = 0x5f3759df - (i >> 1);
            y = *(float *)&i;
            y = y * (1.5f - (halfnum * y * y));
            return y;
        }
        /**
          * @brief          sqrt function
          * @param[in]      val
          * @retval         return sqrt(val)
          */
        template <typename T>
        float sqrt(T val)
        {
            return 1 / rsr(val);
        }

        template <typename T>
        T pow(T base, T exponent);

        template <typename T>
        T exp(T val);
    }

    // Clamp Function
    namespace clamp
    {
        /**
          * @brief          Limiting function
          * @param[in]      val The value to be limited
          * @param[in]      min
          * @param[in]      max
          * @retval         the value after limitation
          */
        template <typename T>
        T val_limit(T val, T min, T max)
        {
            return (val < min) ? min :
                   ((val > max) ? max : val);
        }

        /**
          * @brief          Loop Limiting function
          * @param[in]      val The value to be limited
          * @param[in]      min
          * @param[in]      max
          * @retval         the value after limitation
          */
        template <typename T>
        T val_loop_limit(T val, T min, T max)
        {
            if (max < min)
                return val;

            if (val > max)
            {
                float len = max - min;

                while (val > max)
                    val -= len;
            }
            else if (val < min)
            {
                float len = max - min;

                while (val < min)
                    val += len;
            }
            return val;
        }

        /**
          * @brief          angle Loop Limiting function
          * @param[in]      angle The value to be limited uint:radians
          * @param[in]      min
          * @param[in]      max
          * @retval         the value after limitation
          */
        template <typename T>
        T rad_angle_loop_limit(T angle)
        {
            return val_loop_limit(angle, -M_PI, M_PI);
        }

        /**
         * @brief Map the input value linearly from the original interval to the target interval
         *        将输入值从原始区间线性映射到目标区间
         * @param input_value
         * @param input_min
         * @param input_max
         * @param output_min
         * @param output_max
         * @return the value after mapping
         *
         * @note 当输入区间无效（input_max == input_min）时，返回output_min
         */
        template <typename T>
        T val_remap(T input_value, T input_min, T input_max,
                                 T output_min, T output_max)
        {
            float scale = (output_max - output_min) / (input_max - input_min);
            float result = output_min + (input_value - input_min) * scale;

            if(result > output_max)
                return output_max;

            else if(result < output_min)
                return output_min;

            else
                return result;

        }

        /**
          * @brief          Sign function
          * @param[in]      val The value to be judged
          * @param[in]      min
          * @param[in]      max
          * @retval         the value after judged
          */
        template <typename T>
        int8_t val_sign(T val)
        {
            return(val >= 0) ? 1 : -1;
        }

    }

    // Trigonometric Function
    namespace trig
    {
        /**
          * @brief          sin function
          * @param[in]      rad
          * @retval         sin(rad)
          */
        inline float sin(float rad)
        {
            return arm_sin_f32(rad);
        }

        /**
          * @brief          cos function
          * @param[in]      rad
          * @retval         cos(rad)
          */
        inline float cos(float rad)
        {
            return arm_cos_f32(rad);
        }

        /**
          * @brief          tan function
          * @param[in]      rad
          * @retval         tan(rad)
          */
        inline float tan(float rad)
        {
            return arm_sin_f32(rad) / arm_cos_f32(rad);
        }

        float arcsin(float val);

        float arccos(float val);

        float arctan(float val);
    }

    // Linear Function Build
    namespace linear
    {
        class Ramp
        {
        public:

            /**
            * @brief          slope function initializing
            * @param[in]      period uint:seconds
            * @retval         none
            */
            Ramp& init(float period)
            {
                /* clearing and initializing */
                current = 0.0f;
                target  = 0.0f;
                slope   = 0.0f;

                /* copy period */
                this->frame_period = period;

                return *this;
            }

            /**
            * @brief          slope function calculation
            * @param[in]      _target_
            * @param[in]      _slope_
            * @retval         none
            */
            Ramp& calc(float target, float slope)
            {
                this->target = target;
                float delta = target - current; // calculate delta
                float step = fabsf(slope);      // Take the absolute value of the slope as the step

                if (delta > 0)
                {
                    // Approaching the target value
                    this->current += fminf(delta, step);
                }
                else if (delta < 0)
                {
                    // Approaching the minus target value
                    this->current -= fminf(-delta, step);
                }

                // delta = 0 do nothing

                return *this;
            }

            template<typename T>
            auto get(T member_ptr) -> decltype(auto) {
                return this->*member_ptr; // 返回对应成员的值
            }

//            float get_current()
//            {
//                return current;
//            }

            float current;          // 当前值

        private:
            float target;           // 目标值
            float frame_period;     // 时间间隔
            float slope;            // 变化斜率
        };
    }


    // Filter Function
    namespace filter
    {
        class LowPassFilter;

        float low_pass_filter(float val, float prev_val, float alpha);

        float high_pass_filter(float val, float prev_val, float alpha);
    }

}


class Alg_Utilities {

};

#endif

#endif
