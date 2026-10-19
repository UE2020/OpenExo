#ifndef FSR_THRESHOLD_UART_H
#define FSR_THRESHOLD_UART_H
#include "FsrThresholds.h"
#include "AnklePerturbationSupport.h"

namespace fsr_thresholds
{
    template<class Data> uint8_t permission(Data* data)
    {
        float enabled[1]={0};
        data->for_each_joint([](JointData* joint,float* args){
            if(joint->is_used && joint->motor.enabled)args[0]=1;
        },enabled);
        if(!data->user_paused || enabled[0])return 1;
        if(data->perturbation.busy())return 2;
        if(ankle_perturbation::calibrating(data->left_side) ||
            ankle_perturbation::calibrating(data->right_side))return 4;
        return 0;
    }

    template<class Data> Snapshot capture(Data* data,uint16_t id,uint8_t state,uint8_t reason)
    {
        Snapshot s;s.id=id;s.state=state;s.reason=reason;s.editable=permission(data)==0;
        const float values[4]={data->left_side.toe_fsr_lower_threshold,data->left_side.toe_fsr_upper_threshold,
            data->right_side.toe_fsr_lower_threshold,data->right_side.toe_fsr_upper_threshold};
        for(uint8_t i=0;i<4;++i)if(!isfinite(values[i]) || values[i]<0 || values[i]>1)
        {s.state=3;s.reason=3;s.editable=0;return s;}
        s.ll=lroundf(values[0]*100);s.lu=lroundf(values[1]*100);
        s.rl=lroundf(values[2]*100);s.ru=lroundf(values[3]*100);
        return s;
    }

    template<class Handler,class Data> bool handle_uart(Handler* handler,Data* data,const UART_msg_t& msg)
    {
        if(msg.command==query)
        {
            if(msg.len==0)handler->UART_msg(pack(Snapshot{}));
            return true;
        }
        if(msg.command!=request)return false;
        if(msg.len!=4 || !ankle_perturbation::wire::integer(msg.data[0],255))return true;
        const uint16_t id=uint16_t(msg.joint_id)*256+(uint16_t)msg.data[0];
        if(id<1 || id>9999 || !ankle_perturbation::wire::integer(msg.data[1],1))return true;
        const bool apply=msg.data[1]==1;
        uint8_t reason=apply?permission(data):0;
        if(apply && (!ankle_perturbation::wire::integer(msg.data[2],94) || msg.data[2]<6 ||
                     !ankle_perturbation::wire::integer(msg.data[3],94) || msg.data[3]<6))reason=3;
        if(apply && !reason)
        {
            const float left=msg.data[2]/100,right=msg.data[3]/100;
            data->left_side.toe_fsr_lower_threshold=left-band;
            data->left_side.toe_fsr_upper_threshold=left+band;
            data->right_side.toe_fsr_lower_threshold=right-band;
            data->right_side.toe_fsr_upper_threshold=right+band;
            data->perturbation.require_unload();
        }
        handler->UART_msg(pack(capture(data,id,reason?3:(apply?2:1),reason)));
        return true;
    }
}
#endif
