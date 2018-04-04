
#include "F28x_Project.h"

__interrupt void epwm2_isr(void);

int PWMprd = 20000;
int dir = 1;
int enable = 1;
int cnt = 0;
int cntmax = 50;
float ff = 0.5;

void main(void)
{
    // Uint16 acqps;

    InitSysCtrl();
    InitGpio();

    EALLOW;
    /***** GPIO Settings *****/
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0b00;  // GPIO10 MUX Option 0 : GPIO10
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0b1;    // GPIO10 Set as Output
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0b00;  // GPIO10 MUX Option 0 : GPIO11
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 0b1;    // GPIO11 Set as Output

    /***** EPWM Settings *****/
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;       // EPWM2 Clock Enable
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;      // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;      // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     // Configure GPIO3 as EPWM2B

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // EPWM Time Base Clock Sync OFF

    EPwm2Regs.TBCTL.bit.CTRMODE = 0b00;         // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = 0;              // Disable phase loading
    EPwm2Regs.TBCTL.bit.CLKDIV = 0b000;         // CLKDIV = 1
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0b000;      // HSPCLKDIV = 10
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;         // Phase is 0
    EPwm2Regs.TBPRD = PWMprd;                    // Set timer period
    EPwm2Regs.TBCTR = 0x0000;                   // Clear counter

    // Setup shadow register load on ZERO
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = 0;

    EPwm2Regs.CMPA.bit.CMPA = 0;      // Set compare A value
    EPwm2Regs.CMPB.bit.CMPB = 0;      // Set Compare B value

    EPwm2Regs.AQCTLA.bit.PRD = 0b10;    // Set PWM2A on Period
    EPwm2Regs.AQCTLA.bit.CAU = 0b01;    // Clear PWM2A on event A, up count
    EPwm2Regs.AQCTLB.bit.PRD = 0b10;    // Set PWM2B on Period
    EPwm2Regs.AQCTLB.bit.CBU = 0b01;    // Clear PWM2B on event B, up count

    EPwm2Regs.ETSEL.bit.INTEN = 1;
    EPwm2Regs.ETSEL.bit.INTSEL = 0b010; // Event at counter = prd
    EPwm2Regs.ETPS.bit.INTPRD = 0b01;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // EPWM Time Base Clock Sync ON
    EDIS;

    /***** PIE Settings *****/
    DINT;                       // Clear all interrupts and initialize PIE vector table:
    InitPieCtrl();              // Initialize the PIE control registers to their default state.
    IER = 0x0000; IFR = 0x0000; // Disable CPU interrupts and clear all CPU interrupt flags:
    InitPieVectTable();         // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

    EALLOW;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    EDIS;

    IER |= M_INT3; // Enable INT3
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1; // Enable INT3.2 = EPWM2

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    GpioDataRegs.GPASET.bit.GPIO10 = 0b1;

    while(1)
    {

    }
}

__interrupt void epwm2_isr(void)
{
    if (cnt >= cntmax)
        cnt = 0;
    else if (cnt < (int)(cntmax * ff))
    {
        EPwm2Regs.CMPA.bit.CMPA = 5000;
        cnt++;
    }
    else if (cnt < cntmax)
    {
        EPwm2Regs.CMPA.bit.CMPA = 15000;
        cnt++;
    }
/*
    if (cnt < cntmax)
        cnt++;
    else
    {
        if (dir==1)
        {
            EPwm2Regs.CMPA.bit.CMPA = 5000;
            dir = 0;
        }
        else if (dir==0)
        {
            EPwm2Regs.CMPA.bit.CMPA = 15000;
            dir = 1;
        }
        cnt = 0;
    }
*/
/*
    switch(enable)
    {
    case 0:
        GpioDataRegs.GPACLEAR.bit.GPIO10 = 0b1; break;
    case 1:
        GpioDataRegs.GPASET.bit.GPIO10 = 0b1; break;
    default:
        GpioDataRegs.GPACLEAR.bit.GPIO10 = 0b1; break;
    }
*/
    EPwm2Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
