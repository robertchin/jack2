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

#ifndef __JackAtomicState__
#define __JackAtomicState__

#include "systemdeps.h"
#include "JackTypes.h"
#include <string.h> // for memcpy
#include <atomic>

namespace Jack
{

/*!
\brief Counter for CAS
*/

PRE_PACKED_STRUCTURE
struct AtomicCounter
{
private:
    union _info {
        struct {
            UInt16 fShortVal1;	// Cur
            UInt16 fShortVal2;	// Next
        }
        scounter;
        UInt32 fLongVal;
    };

    std::atomic<union _info> info;

public:
    AtomicCounter()
    {
        union _info t{};
        info.store(t, std::memory_order_seq_cst);
    }

    AtomicCounter(const AtomicCounter& obj)
    {
	info.store(obj.info, std::memory_order_seq_cst);
    }

    AtomicCounter(AtomicCounter& obj)
    {
        union _info v;
        v.fLongVal = obj.Counter();
        info.store(v, std::memory_order_seq_cst);
    }

    AtomicCounter& operator=(AtomicCounter& obj)
    {
        union _info v;
        v.fLongVal = obj.Counter();
        info.store(v, std::memory_order_seq_cst);
        return *this;
    }

    UInt32 Counter() const { return info.load().fLongVal; }
    UInt16 CurIndex() const { return info.load().scounter.fShortVal1; }
    void SetCurIndex(UInt16 val) {
	union _info v = info.load();
	v.scounter.fShortVal1 = val;
	info.store(v);
    }
    UInt16 NextIndex() const { return info.load().scounter.fShortVal2; }
    void SetNextIndex(UInt16 val) {
        union _info v = info.load();
        v.scounter.fShortVal2 = val;
        info.store(v);
    }
    UInt16 CurArrayIndex() const { return CurIndex() & 0x0001; }
    UInt16 NextArrayIndex() const { return (CurIndex() + 1) & 0x0001; }

    bool CompareExchange(AtomicCounter &old, AtomicCounter &repl) {
        auto expected = old.info.load();
        auto value = repl.info.load();
        return info.compare_exchange_strong(expected, value);
    }
} POST_PACKED_STRUCTURE;


/*!
\brief A class to handle two states (switching from one to the other) in a lock-free manner
*/

// CHECK livelock

PRE_PACKED_STRUCTURE
template <class T>
class JackAtomicState
{

    protected:

        T fState[2];
        AtomicCounter fCounter{};
        SInt32 fCallWriteCounter;

        UInt32 WriteNextStateStartAux()
        {
            AtomicCounter old_val;
            AtomicCounter new_val;
            UInt32 cur_index;
            UInt32 next_index;
            bool need_copy;
            do {
                old_val = fCounter;
                new_val = old_val;
                cur_index = new_val.CurIndex();
                next_index = new_val.NextArrayIndex();
                need_copy = new_val.CurIndex() == new_val.NextIndex();
                new_val.SetNextIndex(new_val.CurIndex()); // Invalidate next index
            } while (!fCounter.CompareExchange(old_val,new_val));
            if (need_copy)
                memcpy(&fState[next_index], &fState[cur_index], sizeof(T));
            return next_index;
        }

        void WriteNextStateStopAux()
        {
            AtomicCounter old_val;
            AtomicCounter new_val;
            do {
                old_val = fCounter;
                new_val = old_val;
                new_val.SetNextIndex(new_val.NextIndex() + 1); // Set next index
            } while (!fCounter.CompareExchange(old_val, new_val));
        }

    public:

        JackAtomicState()
        {
            fCallWriteCounter = 0;
        }

        ~JackAtomicState() // Not virtual ??
        {}

        /*!
        \brief Returns the current state : only valid in the RT reader thread
        */
        T* ReadCurrentState()
        {
            return &fState[fCounter.CurArrayIndex()];
        }

        /*!
        \brief Returns the current state index
        */
        UInt16 GetCurrentIndex()
        {
            return fCounter.CurIndex();
        }

        /*!
        \brief Tries to switch to the next state and returns the new current state (either the same as before if case of switch failure or the new one)
        */
        T* TrySwitchState()
        {
            AtomicCounter old_val;
            AtomicCounter new_val;
            do {
                old_val = fCounter;
                new_val = old_val;
                new_val.SetCurIndex(new_val.NextIndex());  // Prepare switch
            } while (!fCounter.CompareExchange(old_val, new_val));
            return &fState[fCounter.CurArrayIndex()];	// Read the counter again
        }

        /*!
        \brief Tries to switch to the next state and returns the new current state (either the same as before if case of switch failure or the new one)
        */
        T* TrySwitchState(bool* result)
        {
            AtomicCounter old_val;
            AtomicCounter new_val;
            do {
                old_val = fCounter;
                new_val = old_val;
                *result = (new_val.CurIndex() != new_val.NextIndex());
                new_val.SetCurIndex(new_val.NextIndex());  // Prepare switch
            } while (!fCounter.CompareExchange(old_val, new_val));
            return &fState[fCounter.CurArrayIndex()];	// Read the counter again
        }

        /*!
        \brief Start write operation : setup and returns the next state to update, check for recursive write calls.
        */
        T* WriteNextStateStart()
        {
            UInt32 next_index = (fCallWriteCounter++ == 0)
                                ? WriteNextStateStartAux()
                                : fCounter.NextArrayIndex(); // We are inside a wrapping WriteNextStateStart call, NextArrayIndex can be read safely
            return &fState[next_index];
        }

        /*!
        \brief Stop write operation : make the next state ready to be used by the RT thread
        */
        void WriteNextStateStop()
        {
            if (--fCallWriteCounter == 0)
                WriteNextStateStopAux();
        }

        bool IsPendingChange()
        {
            return fCounter.CurIndex() != fCounter.NextIndex();
        }

        /*
              // Single writer : write methods get the *next* state to be updated
        void TestWriteMethod()
        {
        	T* state = WriteNextStateStart();
        	......
        	......
        	WriteNextStateStop(); 
        }

              // First RT call possibly switch state
        void TestReadRTMethod1()
        {
        	T* state = TrySwitchState();
        	......
        	......
        }

              // Other RT methods can safely use the current state during the *same* RT cycle
        void TestReadRTMethod2() 
        {
        	T* state = ReadCurrentState();
        	......
        	......
        }

              // Non RT read methods : must check state coherency
        void TestReadMethod()
        {
        	T* state;
        	UInt16 cur_index;
            UInt16 next_index = GetCurrentIndex();
        	do {
                cur_index = next_index;
        		state = ReadCurrentState();
        		
        		......
        		......

                next_index = GetCurrentIndex();
        	} while (cur_index != next_index);
        }
        */

} POST_PACKED_STRUCTURE;

} // end of namespace

#endif

