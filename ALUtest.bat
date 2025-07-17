call "A:\Xilinx\2025.1\Vivado\settings64.bat"

set folder=Simulation_Files
mkdir %folder%
copy "RAM.mem" "%folder%/RAM.mem"
cd "%folder%

::Arithmetic Logic Unit Simulation
call xvlog ../ArithmeticLogicUnit.v  
call xvlog ../ArithmeticLogicUnitSimulation.v
call xvlog ../Helper.v
call xelab -top ArithmeticLogicUnitSimulation -snapshot alusim -debug typical
call xsim alusim -R

::Arithmetic Logic Unit System Simulation
call xvlog ../Register16bit.v  
call xvlog ../Register32bit.v  
call xvlog ../RegisterFile.v
call xvlog ../AddressRegisterFile.v  
call xvlog ../InstructionRegister.v
call xvlog ../DataRegister.v  
call xvlog ../ArithmeticLogicUnit.v
call xvlog ../Memory.v  
call xvlog ../ArithmeticLogicUnitSystem.v  
call xvlog ../ArithmeticLogicUnitSystemSimulation.v
call xvlog ../Helper.v

call xelab -top ArithmeticLogicUnitSystemSimulation -snapshot alusyssim -debug typical
call xsim alusyssim -R

cd ..
