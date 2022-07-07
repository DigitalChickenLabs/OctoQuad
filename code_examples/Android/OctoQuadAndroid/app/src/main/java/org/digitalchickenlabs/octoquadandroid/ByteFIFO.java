/*
 * Copyright (c) 2022 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.digitalchickenlabs.octoquadandroid;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

public class ByteFIFO
{
    private final byte[] buffer;
    private final int capacity;
    private ReentrantLock lock;
    private Condition elementPulled;
    private Condition elementPushed;
    private int head = 0;
    private int tail = 0;
    private volatile boolean shuttingDown = false;

    public static class ShutdownException extends Exception {};

    /**
     * Construct a new ByteFIFO instance
     * @param capacity the number of bytes the FIFO can hold
     */
    public ByteFIFO(int capacity)
    {
        buffer = new byte[capacity+1]; // We need one overhead element, because we don't want the tail to collide with the head when full
        this.capacity = capacity;
        lock = new ReentrantLock();

        elementPulled = lock.newCondition();
        elementPushed = lock.newCondition();
    }

    public void shutdown()
    {
        lock.lock();

        shuttingDown = true;

        // Wake up any waiters
        elementPulled.signalAll();
        elementPushed.signalAll();

        lock.unlock();
    }

    /**
     * Push in an entire byte array, blocking until the operation is complete
     * @param in the byte array to push
     * @throws InterruptedException if the thread was interrupted while waiting for enough empty space
     * @throws IndexOutOfBoundsException if count > capacity
     */
    public void push(byte[] in) throws InterruptedException, ShutdownException
    {
        push(in, in.length, 0);
    }

    /**
     * Push in some bytes from a byte array, blocking until the operation is complete
     * @param in the buffer from which to push bytes
     * @param count the number of bytes to push
     * @param offset starting index of desired push region of input buffer
     * @throws InterruptedException if the thread was interrupted while waiting for enough empty space
     * @throws IndexOutOfBoundsException if count > capacity
     */
    public void push(byte[] in, int count, int offset) throws InterruptedException, ShutdownException
    {
        if(count > capacity)
        {
            throw new IndexOutOfBoundsException("Count cannot be larger than capacity!");
        }

        // Acquire the mutex
        lock.lock();

        try
        {
            if(shuttingDown)
            {
                throw new ShutdownException();
            }

            // Wait until there's enough available space
            while (freeSpaceInternal() < count)
            {
                // This unlocks the mutex while awaiting the condition, thus allowing
                // someone else to grab the mutex and pull something. When this call
                // returns, the mutex is already re-locked for us
                elementPulled.await();
            }

            // Copy in the things
            for(int i = offset; i < offset+count; i++)
            {
                // If the tail hits the end of the buffer, wrap back around to the start.
                // We needn't worry about colliding with the head, since we should only
                // ever be copying the things after there is sufficient space
                if(tail >= buffer.length)
                {
                    tail = 0;
                }

                // Copy in a thing and increment the tail
                buffer[tail] = in[i];
                tail++;
            }

            // Notify anyone who may be waiting to pull things that new things have
            // just been pushed in
            elementPushed.signal();
        }
        finally
        {
            // Release the mutex
            lock.unlock();
        }
    }

    /**
     * Push in an entire byte array, overwriting however much of the oldest
     * data is necessary in order to make the new data fit
     * @param in the byte array to push
     * @throws IndexOutOfBoundsException if count > capacity
     */
    public void pushOverwriting(byte[] in)
    {
        pushOverwriting(in, in.length, 0);
    }

    /**
     * Push in some bytes from a byte array, overwriting however much of the oldest
     * data is necessary in order to make the new data fit
     * @param in the buffer from which to push bytes
     * @param count the number of bytes to push
     * @param offset starting index of desired push region of input buffer
     * @throws IndexOutOfBoundsException if count > capacity
     */
    public void pushOverwriting(byte[] in, int count, int offset)
    {
        if(count > capacity)
        {
            throw new IndexOutOfBoundsException("Count cannot be larger than capacity!");
        }

        // Acquire the mutex
        lock.lock();

        // If there's not enough free space, overwrite the oldest data
        if (freeSpaceInternal() < count)
        {
            int countToRemove = count - freeSpaceInternal();

            if(head+countToRemove >= buffer.length)
            {
                // The head would hit the end of the buffer, so we need to wrap
                // back around to the start
                head = head - buffer.length;
            }
            else
            {
                // If the head won't hit the end of the buffer, that's easy
                head += countToRemove;
            }
        }

        // Now that we've cleared space, copy in the things
        for(int i = offset; i < offset+count; i++)
        {
            // If the tail hits the end of the buffer, wrap back around to the start.
            // We needn't worry about colliding with the head, since we should only
            // ever be copying the things after there is sufficient space
            if(tail >= buffer.length)
            {
                tail = 0;
            }

            // Copy in a thing and increment the tail
            buffer[tail] = in[i];
            tail++;
        }

        // Notify anyone who may be waiting to pull things that new things have
        // just been pushed in
        elementPushed.signal();

        // Release the mutex
        lock.unlock();
    }

    /**
     * Pull out a byte array of a certain length, blocking until the operation is complete
     * @param count the number of bytes to pull out
     * @return an array containing the pulled bytes
     * @throws InterruptedException if the thread was interrupted while waiting for enough upstream data
     * @throws IndexOutOfBoundsException if count > capacity
     */
    public byte[] pull(int count) throws InterruptedException, ShutdownException
    {
        byte[] out = new byte[count];
        pull(out, count, 0);
        return out;
    }

    /**
     * Pull out a certain number of bytes, placing them into an output buffer, and blocking until the operation is complete
     * @param out output buffer
     * @param count number of bytes to pull
     * @param offset starting index to place pulled bytes into output buffer
     * @throws InterruptedException if the thread was interrupted while waiting for enough upstream data
     * @throws IndexOutOfBoundsException if count > capacity
     */
    public void pull (byte[] out, int count, int offset) throws InterruptedException, ShutdownException
    {
        if(count > capacity)
        {
            throw new IndexOutOfBoundsException("Count cannot be larger than capacity!");
        }

        // Acquire the mutex
        lock.lock();

        try
        {
            if(shuttingDown)
            {
                throw new ShutdownException();
            }

            // Wait until we can grab the specified number of things
            while(usedSpaceInternal() < count)
            {
                // This unlocks the mutex while awaiting the condition, thus allowing
                // someone else to grab the mutex and push something. When this call
                // returns, the mutex is already re-locked for us
                elementPushed.await();
            }

            // Pull out the things
            for(int i = offset; i < offset+count; i++)
            {
                // If the head hits the end of the buffer, wrap back around to the start.
                // We needn't worry about colliding with the tail, since we should only
                // ever be pulling the things out after there are at least that many things present.
                if(head >= buffer.length)
                {
                    head = 0;
                }

                // Copy out a thing and increment the tail
                out[i] = buffer[head];
                head++;
            }

            // Notify anyone who may be waiting to push things that something has just been pulled out
            elementPulled.signal();
        }
        finally
        {
            // Release the mutex
            lock.unlock();
        }
    }

    /**
     * Get the capacity of this FIFO
     * @return the capacity of this FIFO
     */
    public int capacity()
    {
        return capacity;
    }

    /**
     * Get remaining capacity (amount of empty space) in this FIFO
     * @return amount of empty space in this FIFO
     */
    public int freeSpace()
    {
        lock.lock();
        int result = freeSpaceInternal();
        lock.unlock();
        return result;
    }

    /**
     * Get the amount of data currently in the FIFO
     * @return the amount of data currently in the FIFO
     */
    public int usedSpace()
    {
        lock.lock();
        int result = usedSpaceInternal();
        lock.unlock();
        return result;
    }

    private int freeSpaceInternal()
    {
        //return buffer.length - usedSpaceInternal() - 1; // We can't let the head collide with the tail
        return capacity - usedSpaceInternal();
    }

    private int usedSpaceInternal()
    {
        if(tail >= head)
        {
            // If the head leads the tail, that's easy
            return tail - head;
        }
        else // head > tail
        {
            // If the tail leads the head, then we need to sum the distance between then head
            // and the end of the buffer with the distance between the tail and the start of the buffer.
            return (buffer.length - head) + tail;
        }
    }
}
