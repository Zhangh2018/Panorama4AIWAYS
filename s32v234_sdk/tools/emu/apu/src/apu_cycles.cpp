/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
* All Rights Reserved
*
******************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
/**
* \file APU_cycles.cpp
* \brief APU cycles counter
* \author Igor Aleksandrowicz
* \version
* \date 04-Mach-2014
****************************************************************************/

#include "apu_cycles.hpp"
#include "apu_vec.hpp"
#include "apu_vbool.hpp"

using namespace APEX2;

//macro used for easier and cleaner cycle database creation
#define CYCLES_ENTRY(INDEX, CYCLES) database[INDEX] = (CYCLES);
//function filling the databyse from apu_cycle_database.hpp
static unordered_map<std::string, CycleCounterType> CreateCycleDatabase()
{
  unordered_map<std::string, CycleCounterType> database;
#include "apu_cycle_database.hpp"
  return database;
}

//global counting variables definitions
CycleCounterType APEX2::gVectorCycles = 0;
CycleCounterType APEX2::gVectorOperations = 0;
bool APEX2::gCycleCountPaused = false;
//create the database
unordered_map<std::string, CycleCounterType> APEX2::gCycleDatabase = CreateCycleDatabase();

//user functions implementation
CycleCounterBaseTp APEX2::GetVectorCyclesElapsed()
{
  return gVectorCycles();
}


CycleCounterBaseTp APEX2::GetVectorOperationsElapsed()
{
  return gVectorOperations();
}

double APEX2::GetEstimatedExecutionTime()
{
	double cycles = GetVectorCyclesElapsed();
	double timeEstim = cycles / CLOCK_FREQ;
	return timeEstim;
}

void APEX2::PauseCycleCounting()
{
  gCycleCountPaused = true;
}

void APEX2::ResumeCycleCounting()
{
  gCycleCountPaused = false;
}

//GetVectorTypeString implementation for various types
template <> std::string APEX2::GetVectorTypeString<vbool>()
{
  return "vbool";
}

template <> std::string APEX2::GetVectorTypeString<vec08u>()
{
  return "vec08u";
}

template <> std::string APEX2::GetVectorTypeString<vec08s>()
{
  return "vec08s";
}

template <> std::string APEX2::GetVectorTypeString<vec16u>()
{
  return "vec16u";
}

template <> std::string APEX2::GetVectorTypeString<vec16s>()
{
  return "vec16s";
}

template <> std::string APEX2::GetVectorTypeString<vec32u>()
{
  return "vec32u";
}

template <> std::string APEX2::GetVectorTypeString<vec32s>()
{
  return "vec32s";
}

template <> std::string APEX2::GetVectorTypeString<int08u>()
{
  return "vec08u";
}

template <> std::string APEX2::GetVectorTypeString<int08s>()
{
  return "vec08s";
}

template <> std::string APEX2::GetVectorTypeString<int16u>()
{
  return "vec16u";
}

template <> std::string APEX2::GetVectorTypeString<int16s>()
{
  return "vec16s";
}

template <> std::string APEX2::GetVectorTypeString<int32u>()
{
  return "vec32u";
}

template <> std::string APEX2::GetVectorTypeString<int32s>()
{
  return "vec32s";
}
