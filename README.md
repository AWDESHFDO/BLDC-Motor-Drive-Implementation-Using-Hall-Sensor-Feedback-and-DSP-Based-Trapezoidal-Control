
#include "F28x_Project.h"

// =============================
// CONSTANT DEFINITIONS (LAB STEPS 2, 8, 9)
// =============================
#define EPWM_TIMER_TBPRD 4999              // 20 kHz PWM period for EPWM modules (Step 2)
#define CPU_TIMER1_PRD 499999999           // CPU Timer1 → 2.5 sec for speed measurement (Step 8)
#define CPU_TIMER0_PRD 50000000-1          // CPU Timer0 → 250 ms for PI control loop (Step 9)

// =============================
// ISR DECLARATIONS (STEPS 8, 9)
// =============================
interrupt void xint1_isr(void);            // External interrupt ISR for Hall rising edge (speed calc)
interrupt void cpu_timer0_isr(void);       // Timer0 ISR (PI control)

// =============================
// GLOBAL VARIABLES
// =============================
Uint16 numPoles = 8;                       // Number of poles of BLDC motor (Step 8)
int16 speed;                               // Measured BLDC speed (RPM)

// DAC monitor variables
volatile Uint16 dac0 = 200;                // DAC0 output (Step 8)
volatile Uint16 dac1 = 200;                // DAC1 output

// PI Controller variables (Step 9)
int16 ki = 8;                              
int16 kp = 0;
int16 Up = 0;
int16 Ui = 0;
int16 Out = 0;

// Hall sensor inputs (Step 3)
int16 H1 = 0;
int16 H2 = 0;
int16 H3 = 0;

// Speed reference and error (Step 9)
int16 speed_ref = 600;
int16 speed_error = 0;

// Initial 50% duty for PWM control (Step 4)
Uint16 control_H = 0.5*EPWM_TIMER_TBPRD;


// =======================================
// EPWM1 INITIALIZATION (SUPPLIES PHASE A)
// Step 2
// =======================================
void InitEPwm1Example()
{
    // Configure GPIO0/1 as EPWM1A/B
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;        // EPWM1A output
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;        // EPWM1B output
    EDIS;

    // Set EPWM1 for 20 kHz up-count PWM (Step 2)
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;           
    EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;        
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;         
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;            

    // Initial duty (0%) for A/B outputs
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.CMPB.bit.CMPB = 0;

    // Use shadow registers (update on zero)
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Action qualifier: set at ZERO, clear at compare (Step 2)
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}


// =======================================
// EPWM2 INITIALIZATION (SUPPLIES PHASE B)
// Step 2
// =======================================
void InitEPwm2Example()
{
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;    // EPWM2A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;    // EPWM2B
    EDIS;

    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;

    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}


// =======================================
// EPWM3 INITIALIZATION (SUPPLIES PHASE C)
// Step 2
// =======================================
void InitEPwm3Example()
{
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;    // EPWM3A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;    // EPWM3B
    EDIS;

    EPwm3Regs.TBCTL.bit.CTRMODE = 0;
    EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;

    EPwm3Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPB.bit.CMPB = 0;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}


// =======================================
// EPWM6 INIT (USED FOR DAC OUTPUT → Step 8)
// =======================================
void InitEPwm6Example(void)
{
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // EPWM6A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // EPWM6B
    EDIS;

    EPwm6Regs.TBCTL.bit.CTRMODE = 0;
    EPwm6Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm6Regs.TBCTL.bit.CLKDIV = 0;

    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}


// =======================================
// SIX COMMUTATION SECTOR FUNCTIONS
// Step 4 — Based on Fig.2 of Manual
// =======================================

// Sector 1: (A+, B-)
void sector1()
{
    EPwm1Regs.CMPA.bit.CMPA = control_H;        // A+
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPA.bit.CMPA = 0;

    EPwm1Regs.CMPB.bit.CMPB = 0;                
    EPwm2Regs.CMPB.bit.CMPB = EPWM_TIMER_TBPRD; // B-
    EPwm3Regs.CMPB.bit.CMPB = 0;
}

// Sector 2: (A+, C-)
void sector2()
{
    EPwm1Regs.CMPA.bit.CMPA = control_H;        // A+
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPA.bit.CMPA = 0;

    EPwm1Regs.CMPB.bit.CMPB = 0;                
    EPwm2Regs.CMPB.bit.CMPB = 0;
    EPwm3Regs.CMPB.bit.CMPB = EPWM_TIMER_TBPRD; // C-
}

// Sector 3: (B+, C-)
void sector3()
{
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPA.bit.CMPA = control_H;        // B+
    EPwm3Regs.CMPA.bit.CMPA = 0;

    EPwm1Regs.CMPB.bit.CMPB = 0;                
    EPwm2Regs.CMPB.bit.CMPB = 0;
    EPwm3Regs.CMPB.bit.CMPB = EPWM_TIMER_TBPRD; // C-
}

// Sector 4: (A-, B+)
void sector4()
{
    EPwm1Regs.CMPB.bit.CMPB = EPWM_TIMER_TBPRD; // A-
    EPwm2Regs.CMPB.bit.CMPB = 0; 
    EPwm3Regs.CMPB.bit.CMPB = 0;

    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPA.bit.CMPA = control_H;        // B+
    EPwm3Regs.CMPA.bit.CMPA = 0;
}

// Sector 5: (A-, C+)
void sector5()
{
    EPwm1Regs.CMPB.bit.CMPB = EPWM_TIMER_TBPRD; // A-
    EPwm2Regs.CMPB.bit.CMPB = 0; 
    EPwm3Regs.CMPB.bit.CMPB = 0;

    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPA.bit.CMPA = control_H;        // C+
}

// Sector 6: (B-, C+)
void sector6()
{
    EPwm1Regs.CMPB.bit.CMPB = 0;
    EPwm2Regs.CMPB.bit.CMPB = EPWM_TIMER_TBPRD; // B-
    EPwm3Regs.CMPB.bit.CMPB = 0;

    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPA.bit.CMPA = control_H;        // C+
}


// =======================================
//                MAIN()
// =======================================
void main(void)
{
    InitSysCtrl();
    DINT;
    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    // ------------------------------
    // Step 3 — Configure Hall Sensor GPIOs (GPIO24,25,26)
    // ------------------------------
    EALLOW;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO24 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;

    GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;

    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;
    EDIS;


    // ------------------------------
    // PIE Initialization
    // ------------------------------
    InitPieVectTable();

    // ============================
    // Step 8 — External Interrupt XINT1 Setup (Hall rising edge)
    // ============================
    EALLOW;
    PieVectTable.XINT1_INT = &xint1_isr;
    GPIO_SetupXINT1Gpio(24);                   // Use Hall H1 rising edge
    XintRegs.XINT1CR.bit.POLARITY = 1;         // Rising edge trigger
    XintRegs.XINT1CR.bit.ENABLE = 1;           // Enable XINT1
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;         // Enable PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;         // Enable PIE Group1 INT4
    IER |= M_INT1;                             
    EDIS;

    EINT;   // Enable Global Interrupt

    // Input qualification filtering for Hall sensor (Step 8)
    EALLOW;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 2;
    GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 255;
    EDIS;


    // ============================
    // Step 9 — CPU Timer0 PI Control
    // ============================
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    CpuTimer0Regs.TCR.bit.TSS = 1;
    CpuTimer0Regs.PRD.all = CPU_TIMER0_PRD;
    CpuTimer0Regs.TCR.bit.TRB = 1;
    CpuTimer0Regs.TCR.bit.TIE = 1;
    CpuTimer0Regs.TCR.bit.TSS = 0;
    EDIS;


    // ============================
    // PWM Initialization (Step 2)
    // ============================
    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm3Example();
    InitEPwm6Example();


    // ============================
    // Step 8 — CPU Timer1 Setup (Speed Calculation)
    // ============================
    CpuTimer1Regs.TCR.bit.TSS = 1;
    CpuTimer1Regs.PRD.all = CPU_TIMER1_PRD;
    CpuTimer1Regs.TCR.bit.TRB = 1;


    // ============================
    // MAIN LOOP (Step 5)
    // ============================
    for(;;)
    {
        // Read Hall sensors
        H1 = GpioDataRegs.GPADAT.bit.GPIO24;
        H2 = GpioDataRegs.GPADAT.bit.GPIO25;
        H3 = GpioDataRegs.GPADAT.bit.GPIO26;

        // Monitor values through DAC (Step 8)
        EPwm6Regs.CMPA.bit.CMPA = dac0;
        EPwm6Regs.CMPB.bit.CMPB = dac1;
        dac0 = speed;
        dac1 = speed_ref;

        // ========================
        // Step 5 — Commutation Logic Based on Fig.2
        // ========================
        if (H1==1 & H2==0 & H3==1) { sector4(); }
        if (H1==1 & H2==0 & H3==0) { sector5(); }
        if (H1==1 & H2==1 & H3==0) { sector6(); }
        if (H1==0 & H2==1 & H3==0) { sector1(); }
        if (H1==0 & H2==1 & H3==1) { sector2(); }
        if (H1==0 & H2==0 & H3==1) { sector3(); }
    }
}



// =======================================
// XINT1 ISR — Step 8
// Compute Speed Using Time Between Hall Edges
// =======================================
interrupt void xint1_isr(void)
{
    speed = 0;

    CpuTimer1Regs.TCR.bit.TSS = 1;

    if (CpuTimer1Regs.TIM.all != CPU_TIMER1_PRD)
    {
        speed = 60*(200000000.0) / (numPoles/2) /
                 (CPU_TIMER1_PRD - CpuTimer1Regs.TIM.all);
    }

    CpuTimer1Regs.TCR.bit.TRB = 1;
    CpuTimer1Regs.TCR.bit.TSS = 0;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



// =======================================
// Timer0 ISR — PI Speed Controller (Step 9)
// =======================================
interrupt void cpu_timer0_isr(void)
{
    speed_error = speed_ref - speed;
    Up = kp * speed_error;
    Ui += speed_error / ki;

    if (Ui > EPWM_TIMER_TBPRD) Ui = EPWM_TIMER_TBPRD;
    else if (Ui < 0) Ui = 0;

    control_H = Up + Ui;

    if (control_H > EPWM_TIMER_TBPRD) control_H = EPWM_TIMER_TBPRD;
    else if (control_H < 0) control_H = 0;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
