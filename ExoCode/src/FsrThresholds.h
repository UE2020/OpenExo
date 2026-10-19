#ifndef FSR_THRESHOLDS_H
#define FSR_THRESHOLDS_H
#include "AnklePerturbationProtocol.h"

// Optional negotiated settings protocol; legacy R/startup behavior is unchanged.
namespace fsr_thresholds
{
    constexpr uint8_t query=0x21, request=0x22, status=0x23;
    constexpr float band=.05f;
    struct Snapshot
    {
        uint16_t id=0;
        uint8_t state=0,reason=0,ll=0,lu=0,rl=0,ru=0,editable=0;
    };

    inline bool center(float v)
    {
        return isfinite(v) && v>=.06f && v<=.94f && fabsf(v*100-roundf(v*100))<.001f;
    }

    inline bool from_ble(const float* v,int n,UART_msg_t& msg)
    {
        if (!v || n!=4 || !ankle_perturbation::wire::integer(v[0],1) ||
            !ankle_perturbation::wire::integer(v[3],9999) || v[3]<1) return false;
        if (v[0]==0 ? (v[1]!=0 || v[2]!=0) : (!center(v[1]) || !center(v[2]))) return false;
        const uint16_t id=(uint16_t)v[3];
        msg=UART_msg_t{};msg.command=request;msg.joint_id=id>>8;msg.len=4;
        msg.data[0]=id&255;msg.data[1]=v[0];
        msg.data[2]=roundf(v[1]*100);msg.data[3]=roundf(v[2]*100);
        return true;
    }

    inline UART_msg_t pack(const Snapshot& s)
    {
        UART_msg_t m{};m.command=status;m.joint_id=s.id>>8;m.len=8;
        const uint8_t values[8]={(uint8_t)(s.id&255),s.state,s.reason,s.ll,s.lu,s.rl,s.ru,s.editable};
        for(uint8_t i=0;i<8;++i)m.data[i]=values[i];
        return m;
    }

    inline bool unpack(const UART_msg_t& m,Snapshot& s)
    {
        if(m.command!=status || m.len!=8)return false;
        for(uint8_t i=0;i<8;++i)if(!ankle_perturbation::wire::integer(m.data[i],255))return false;
        const uint16_t id=uint16_t(m.joint_id)*256+(uint16_t)m.data[0];
        if(id>9999 || m.data[1]>3 || m.data[2]>4 || m.data[7]>1 ||
            m.data[3]>m.data[4] || m.data[4]>100 || m.data[5]>m.data[6] || m.data[6]>100)return false;
        s=Snapshot{id,(uint8_t)m.data[1],(uint8_t)m.data[2],(uint8_t)m.data[3],
            (uint8_t)m.data[4],(uint8_t)m.data[5],(uint8_t)m.data[6],(uint8_t)m.data[7]};
        return true;
    }

    inline int pack_readback(const Snapshot& s,uint8_t* out,size_t size)
    {
        if(!out || size<16)return 0;
        out[0]='F';out[1]='T';out[2]=1;out[3]=s.id&255;out[4]=s.id>>8;
        out[5]=s.state;out[6]=s.reason;
        const uint8_t values[4]={s.ll,s.lu,s.rl,s.ru};
        for(uint8_t i=0;i<4;++i){out[7+2*i]=values[i];out[8+2*i]=0;}
        out[15]=s.editable;return 16;
    }

}
#endif
