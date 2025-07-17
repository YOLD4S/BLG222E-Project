#!/bin/bash

# Source Vivado environment (Linux path!)
source /opt/Xilinx/Vivado/2017.4/settings64.sh

# Create simulation folder
folder="Simulation_Files"
mkdir -p "$folder"
cp RAM.mem "$folder/RAM.mem"
cd "$folder" || exit

# --- Register 16 Simulation ---
xvlog ../Register16bit.v  
xvlog ../Register16bitSimulation.v
xvlog ../Helper.v
xelab -top Register16bitSimulation -snapshot reg16sim -debug typical
xsim reg16sim -R

# --- Register 32 Simulation ---
xvlog ../Register32bit.v  
xvlog ../Register32bitSimulation.v
xvlog ../Helper.v
xelab -top Register32bitSimulation -snapshot reg32sim -debug typical
xsim reg32sim -R

# --- Register File Simulation ---
xvlog ../Register32bit.v  
xvlog ../RegisterFile.v  
xvlog ../RegisterFileSimulation.v
xvlog ../Helper.v
xelab -top RegisterFileSimulation -snapshot regfilesim -debug typical
xsim regfilesim -R

# --- Address Register File Simulation ---
xvlog ../Register16bit.v  
xvlog ../AddressRegisterFile.v  
xvlog ../AddressRegisterFileSimulation.v
xvlog ../Helper.v
xelab -top AddressRegisterFileSimulation -snapshot addregfilesim -debug typical
xsim addregfilesim -R

# --- Instruction Register Simulation ---
xvlog ../InstructionRegister.v  
xvlog ../InstructionRegisterSimulation.v
xvlog ../Helper.v
xelab -top InstructionRegisterSimulation -snapshot insregsim -debug typical
xsim insregsim -R

# --- Data Register Simulation ---
xvlog ../DataRegister.v  
xvlog ../DataRegisterSimulation.v
xvlog ../Helper.v
xelab -top DataRegisterSimulation -snapshot dataregsim -debug typical
xsim dataregsim -R

# --- Arithmetic Logic Unit Simulation ---
xvlog ../ArithmeticLogicUnit.v  
xvlog ../ArithmeticLogicUnitSimulation.v
xvlog ../Helper.v
xelab -top ArithmeticLogicUnitSimulation -snapshot alusim -debug typical
xsim alusim -R

# --- Arithmetic Logic Unit System Simulation ---
xvlog ../Register16bit.v  
xvlog ../Register32bit.v  
xvlog ../RegisterFile.v
xvlog ../AddressRegisterFile.v  
xvlog ../InstructionRegister.v
xvlog ../DataRegister.v  
xvlog ../ArithmeticLogicUnit.v
xvlog ../Memory.v  
xvlog ../ArithmeticLogicUnitSystem.v  
xvlog ../ArithmeticLogicUnitSystemSimulation.v
xvlog ../Helper.v

xelab -top ArithmeticLogicUnitSystemSimulation -snapshot alusyssim -debug typical
xsim alusyssim -R

cd ..