/*
 Copyright (C) 2004-2008 Grame
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software 
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 
 */

#ifndef __JackAtomicArrayState__
#define __JackAtomicArrayState__

#include "systemdeps.h"
#include "JackCompilerDeps.h"
#include "JackTypes.h"
#include <string.h> // for memcpy
#include <atomic>

namespace Jack
{

/*!
\brief Counter for CAS
*/
PRE_PACKED_STRUCTURE
struct AtomicArrayCounter
{
private:
    union _info {
        struct {
            unsigned char fByteVal[4];
        }
        scounter;
        UInt32 fLongVal;
    };

    std::atomic<union _info> info;

public:
    AtomicArrayCounter()
    {
	union _info t{};
        info.store(t, std::memory_order_seq_cst);
    }

    AtomicArrayCounter(const AtomicArrayCounter& obj)
    {
        info.store(obj.info, std::memory_order_seq_cst);
    }

    AtomicArrayCounter(AtomicArrayCounter& obj)
    {
   	union _info v;
        v.fLongVal = obj.Counter1();
        info.store(v, std::memory_order_seq_cst);
    }

    AtomicArrayCounter& operator=(AtomicArrayCounter& obj)
    {
   	union _info v;
        v.fLongVal = obj.Counter1();
        info.store(v, std::memory_order_seq_cst);
        return *this;
    }
       
    UInt32 Counter1() const { return info.load().fLongVal; }
    void SetCounter1(UInt32 value) {
	union _info v = info.load();
 	v.fLongVal = value;
	info.store(v);
    }
    unsigned char GetIndex1(unsigned state) { return info.load().scounter.fByteVal[state]; }
    void SetIndex1(unsigned state, char val) {
	union _info v = info.load();
 	v.scounter.fByteVal[state] = val;
	info.store(v);
    }
    void IncIndex1(unsigned state) {
	union _info v = info.load();
	v.scounter.fByteVal[state]++;
	info.store(v);
    }
    unsigned char SwapIndex1(unsigned state) {
	union _info v = info.load();
	return v.scounter.fByteVal[0] = (v.scounter.fByteVal[0] == state) ? 0 : state;
    }
    bool CompareExchange(AtomicArrayCounter &old, AtomicArrayCounter &repl) {
        auto expected = old.info.load();
        auto value = repl.info.load();
        return info.compare_exchange_strong(expected, value);
    }
} POST_PACKED_STRUCTURE;

/*!
\brief A class to handle several states in a lock-free manner

Requirement:

	- a "current" state
	- several possible "pending" state
	- an TrySwitchState(int state) operation to atomically switch a "pending" to the "current" state (the pending becomes the current).

	The TrySwitchState operation returns a "current" state (either the same if switch fails or the new one, one can know if the switch has succeeded)

	- a WriteNextStartState(int state) returns a "pending" state to be written into
	- a WriteNextStartStop(int state) make the written "pending" state become "switchable"

	Different pending states can be written independently and concurrently.

	GetCurrentIndex() *must* return an increasing value to be able to check reading current state coherency

	The fCounter is an array of indexes to access the current and 3 different "pending" states.

	  WriteNextStateStart(int index) must return a valid state to be written into, and must invalidate state "index" ==> cur state switch.
	  WriteNextStateStop(int index) makes the "index" state become "switchable" with the current state.
	  TrySwitchState(int index) must detect that pending state is a new state, and does the switch
	  ReadCurrentState() must return the state
	  GetCurrentIndex() must return an index increased each new switch.
	  WriteNextStateStart(int index1) and WriteNextStateStart(int index2) can be interleaved

	[switch counter][index state][index state][cur index]

*/

// CHECK livelock

PRE_PACKED_STRUCTURE
template <class T>
class JackAtomicArrayState
{

    protected:

        // fState[0] ==> current
        // fState[1] ==> pending
        // fState[2] ==> request

        T fState[3];
        AtomicArrayCounter fCounter{};

        UInt32 WriteNextStateStartAux(int state, bool* result)
        {
            AtomicArrayCounter old_val;
            AtomicArrayCounter new_val;
            UInt32 cur_index;
            UInt32 next_index;
            bool need_copy;
            do {
                old_val = fCounter;
                new_val = old_val;
                *result = new_val.GetIndex1(state);
                cur_index = new_val.GetIndex1(0);
                next_index = fCounter.SwapIndex1(state);
                need_copy = (new_val.GetIndex1(state) == 0);	// Written = false, switch just occurred
                new_val.SetIndex1(state, 0);					// Written = false, invalidate state
            } while (!fCounter.CompareExchange(old_val, new_val));
            if (need_copy)
                memcpy(&fState[next_index], &fState[cur_index], sizeof(T));
            return next_index;
        }

        void WriteNextStateStopAux(int state)
        {
            AtomicArrayCounter old_val;
            AtomicArrayCounter new_val;
            do {
                old_val = fCounter;
                new_val = old_val;
                new_val.SetIndex1(state, 1);  // Written = true, state becomes "switchable"
            } while (!fCounter.CompareExchange(old_val, new_val));
        }

    public:

        JackAtomicArrayState()
        {
            fCounter.SetCounter1(0);
        }

        ~JackAtomicArrayState() // Not virtual ??
        {}

        /*!
        \brief Returns the current state : only valid in the RT reader thread 
        */

        T* ReadCurrentState()
        {
            return &fState[fCounter.GetIndex1(0)];
        }

        /*!
        \brief Returns the current switch counter
        */

        UInt16 GetCurrentIndex()
        {
            return fCounter.GetIndex1(3);
        }

        /*!
        \brief Tries to switch to the next state and returns the new current state (either the same as before if case of switch failure or the new one)
        */

        T* TrySwitchState(int state)
        {
            AtomicArrayCounter old_val;
            AtomicArrayCounter new_val;
            do {
                old_val = fCounter;
                new_val = old_val;
                if (new_val.GetIndex1(state)) {						// If state has been written
                    new_val.SetIndex1(0, new_val.SwapIndex1(state));	// Prepare switch
                    new_val.SetIndex1(state, 0);						// Invalidate the state "state"
                    new_val.IncIndex1(3);								// Inc switch
                }
            } while (!fCounter.CompareExchange(old_val, new_val));
            return &fState[fCounter.GetIndex1(0)];	// Read the counter again
        }

        /*!
        \brief Tries to switch to the next state and returns the new current state (either the same as before if case of switch failure or the new one)
        */

        T* TrySwitchState(int state, bool* result)
        {
            AtomicArrayCounter old_val;
            AtomicArrayCounter new_val;
            do {
                old_val = fCounter;
                new_val = old_val;
                if ((*result = new_val.GetIndex1(state))) {			// If state has been written
                    new_val.SetIndex1(0, new_val.SwapIndex1(state));	// Prepare switch
                    new_val.SetIndex1(state, 0);						// Invalidate the state "state"
                    new_val.IncIndex1(3);								// Inc switch
                }
            } while (!fCounter.CompareExchange(old_val, new_val));
            return &fState[fCounter.GetIndex1(0)];	// Read the counter again
        }

        /*!
        \brief Start write operation : setup and returns the next state to update, check for recursive write calls.
        */

        T* WriteNextStateStart(int state)
        {
            bool tmp;
            UInt32 index = WriteNextStateStartAux(state, &tmp);
            return &fState[index];
        }

        T* WriteNextStateStart(int state, bool* result)
        {
            UInt32 index = WriteNextStateStartAux(state, result);
            return &fState[index];
        }

        /*!
        \brief Stop write operation : make the next state ready to be used by the RT thread
        */

        void WriteNextStateStop(int state)
        {
            WriteNextStateStopAux(state);
        }

} POST_PACKED_STRUCTURE;

} // end of namespace


#endif

