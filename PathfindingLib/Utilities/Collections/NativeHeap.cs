﻿// MIT License
//
// Copyright(c) 2020 Alex Marcolina
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

using static Unity.Collections.LowLevel.Unsafe.UnsafeUtility;

namespace PathfindingLib.Utilities.Collections;

internal struct NativeHeapIndex
{
    internal static readonly NativeHeapIndex Invalid = new() { TableIndex = -1 };

    internal int TableIndex;
#if ENABLE_UNITY_COLLECTIONS_CHECKS
    internal int Version;
    internal int StructureId;
#endif

    internal readonly bool IsValid => TableIndex >= 0;
}

/// <summary>
/// This is a basic implementation of the MinHeap/MaxHeap data structure.  It allows you
/// to insert objects into the container with a O(log(n)) cost per item, and it allows you
/// to extract the min/max from the container with a O(log(n)) cost per item.
/// 
/// This implementation provides the ability to remove items from the middle of the container
/// as well.  This is a critical operation when implementing algorithms like a-star.  When an
/// item is added to the container, an index is returned which can be used to later remove
/// the item no matter where it is in the heap, for the same cost of removing it if it was
/// popped normally.
/// 
/// This container is parameterized with a comparator type that defines the ordering of the
/// container.  The default form of the comparator can be used, or you can specify your own.
/// The item that comes first in the ordering is the one that will be returned by the Pop
/// operation.  This allows you to use the comparator to parameterize this collection into a 
/// MinHeap, MaxHeap, or other type of ordered heap using your own custom type.
/// 
/// For convenience, this library contains the Min and Max comparator, which provide
/// comparisons for all built in primitives.
/// </summary>
[NativeContainer]
[DebuggerDisplay("Count = {Count}")]
[StructLayout(LayoutKind.Sequential)]
internal struct NativeHeap<T, U> : IDisposable
    where T : unmanaged
    where U : unmanaged, IComparer<T>
{
    #region API

    internal const int DEFAULT_CAPACITY = 128;

    /// <summary>
    /// Returns the number of elements that this collection can hold before the private structures
    /// need to be reallocated.
    /// </summary>
    internal int Capacity
    {
        get
        {
            unsafe
            {
                return Data->Capacity;
            }
        }
        set
        {
            unsafe
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
                if (value < Data->Count) {
                    throw new ArgumentException($"Capacity of {value} cannot be smaller than count of {Data->Count}.");
                }
#endif
                TableValue* newTable = (TableValue*)Malloc(SizeOf<TableValue>() * value, AlignOf<TableValue>(), Allocator);
                HeapNode<T>* newHeap = (HeapNode<T>*)Malloc(SizeOf<HeapNode<T>>() * value, AlignOf<HeapNode<T>>(), Allocator);

                int toCopy = Data->Capacity < value ? Data->Capacity : value;
                MemCpy(newTable, Data->Table, toCopy * SizeOf<TableValue>());
                MemCpy(newHeap, Data->Heap, toCopy * SizeOf<HeapNode<T>>());

                for (int i = 0; i < value - Data->Capacity; i++)
                {
                    //For each new heap node, make sure that it has a new unique index
                    newHeap[i + Data->Capacity] = new HeapNode<T>()
                    {
                        TableIndex = i + Data->Capacity
                    };

#if ENABLE_UNITY_COLLECTIONS_CHECKS
                    //For each new table value, make sure it has a specific version
                    newTable[i + Data->Capacity] = new TableValue() {
                        Version = 1
                    };
#endif
                }

                Free(Data->Table, Allocator);
                Free(Data->Heap, Allocator);

                Data->Table = newTable;
                Data->Heap = newHeap;

                Data->Capacity = value;
            }
        }
    }

    /// <summary>
    /// Returns the number of elements currently contained inside this collection.
    /// </summary>
    internal int Count
    {
        get
        {
            unsafe
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return Data->Count;
            }
        }
    }

    /// <summary>
    /// Gets or sets the comparator used for this Heap. Note that you can only set the comparator
    /// when the Heap is empty.
    /// </summary>
    internal U Comparator
    {
        get
        {
            unsafe
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return Data->Comparator;
            }
        }
        set
        {
            unsafe
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);

                if (Data->Count != 0) {
                    throw new InvalidOperationException("Can only change the comparator of a NativeHeap when it is empty.");
                }
#endif
                Data->Comparator = value;
            }
        }
    }

    /// <summary>
    /// Constructs a new NativeHeap using the given Allocator.  You must call Dispose on this collection
    /// when you are finished with it.
    /// </summary>
    /// <param name="allocator">
    /// You must specify an allocator to use for the creation of the private data structures.
    /// </param>
    /// <param name="initialCapacity">
    /// You can optionally specify the default number of elements this collection can contain before the private
    /// data structures need to be re-allocated.
    /// </param>
    /// <param name="comparator">
    /// You can optionally specify the comparator used to order the elements in this collection.  The Pop operation will
    /// always return the smallest element according to the ordering specified by this comparator.
    /// </param>
    internal NativeHeap(Allocator allocator, int initialCapacity = DEFAULT_CAPACITY, U comparator = default) :
        this(initialCapacity, comparator, allocator, disposeSentinelStackDepth: 1)
    { }

    /// <summary>
    /// Disposes of this container and deallocates its memory immediately.
    /// 
    /// Any NativeHeapIndex structures obtained will be invalidated and cannot be used again.
    /// </summary>
    public void Dispose()
    {
        unsafe
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Dispose(ref m_Safety, ref m_DisposeSentinel);
#endif

            Data->Count = 0;
            Data->Capacity = 0;

            Free(Data->Heap, Allocator);
            Free(Data->Table, Allocator);
            Free(Data, Allocator);
        }
    }

    /// <summary>
    /// Removes all elements from this container.  Any NativeHeapIndex structures obtained will be
    /// invalidated and cannot be used again.
    /// </summary>
    internal void Clear()
    {
        unsafe
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);

            for (int i = 0; i < Data->Count; i++) {
                var node = Data->Heap[i];
                Data->Table[node.TableIndex].Version++;
            }
#endif

            Data->Count = 0;
        }
    }

    /// <summary>
    /// Returns whether or not the given NativeHeapIndex is a valid index for this container.  If true,
    /// that index can be used to Remove the element tied to that index from the container.
    /// 
    /// This method will always return true if Unity safety checks is turned off.
    /// </summary>
    internal bool IsValidIndex(NativeHeapIndex index)
    {
        var isValid = true;
        var errorCode = 0;
        IsValidIndexInternal(index, ref isValid, ref errorCode);
        return isValid;
    }

    /// <summary>
    /// Throws an ArgumentException if the provided NativeHeapIndex is not valid for this container.
    /// 
    /// This method will never throw if Unity safety checks is turned off.
    /// </summary>
    [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
    internal void AssertValidIndex(NativeHeapIndex index)
    {
        var isValid = true;
        var errorCode = 0;
        IsValidIndexInternal(index, ref isValid, ref errorCode);
        if (isValid)
        {
            return;
        }

        switch (errorCode)
        {
            case VALIDATION_ERROR_WRONG_INSTANCE:
                throw new ArgumentException("The provided ItemHandle was not valid for this NativeHeap.  " +
                                            "It was taken from a different instance.");
            case VALIDATION_ERROR_INVALID:
                throw new ArgumentException("The provided ItemHandle was not valid for this NativeHeap.");
            case VALIDATION_ERROR_REMOVED:
                throw new ArgumentException("The provided ItemHandle was not valid for this NativeHeap.  " +
                                            "The item it pointed to might have already been removed.");
        }
    }

    /// <summary>
    /// Returns the next element that would be obtained if Pop was called.  This is the first/smallest
    /// item according to the ordering specified by the comparator.
    /// 
    /// This method is an O(1) operation.
    /// 
    /// This method will throw an InvalidOperationException if the collection is empty.
    /// </summary>
    internal T Peek()
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

        if (!TryPeek(out T t))
        {
            throw new InvalidOperationException("Cannot Peek NativeHeap when the count is zero.");
        }

        return t;
    }

    /// <summary>
    /// Returns the next element that would be obtained if Pop was called.  This is the first/smallest
    /// item according to the ordering specified by the comparator.
    /// 
    /// This method is an O(1) operation.
    /// 
    /// This method will return true if an element could be obtained, or false if the container is empty.
    /// </summary>
    internal bool TryPeek(out T t)
    {
        unsafe
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

            if (Data->Count == 0)
            {
                t = default;
                return false;
            }
            else
            {
                unsafe
                {
                    t = Data->Heap[0].Item;
                    return true;
                }
            }
        }
    }

    /// <summary>
    /// Removes the first/smallest element from the container and returns it.
    /// 
    /// This method is an O(log(n)) operation.
    /// 
    /// This method will throw an InvalidOperationException if the collection is empty.
    /// </summary>
    internal T Pop()
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

        if (!TryPop(out T t))
        {
            throw new InvalidOperationException("Cannot Pop NativeHeap when the count is zero.");
        }

        return t;
    }

    /// <summary>
    /// Removes the first/smallest element from the container and returns it.
    /// 
    /// This method is an O(log(n)) operation.
    /// 
    /// This method will return true if an element could be obtained, or false if the container is empty.
    /// </summary>
    internal bool TryPop(out T t)
    {
        unsafe
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

            if (Data->Count == 0)
            {
                t = default;
                return false;
            }

            var rootNode = Data->Heap[0];

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            // Update version to invalidate all existing handles
            Data->Table[rootNode.TableIndex].Version++;
#endif

            //Grab the last node off the end and remove it
            int lastNodeIndex = --Data->Count;
            var lastNode = Data->Heap[lastNodeIndex];

            //Move the previous root to the end of the array to fill the space we just made
            Data->Heap[lastNodeIndex] = rootNode;

            //Finally insert the previously last node at the root and bubble it down
            InsertAndBubbleDown(lastNode, 0);

            t = rootNode.Item;
            return true;
        }
    }

    /// <summary>
    /// Inserts the provided element into the container.  It may later be removed by a call to Pop,
    /// TryPop, or Remove.
    /// 
    /// This method returns a NativeHeapIndex.  This index can later be used to Remove the item from
    /// the collection.  Once the item is removed by any means, this NativeHeapIndex will become invalid.
    /// If an item is re-added to the collection after it has been removed, Insert will return a NEW
    /// index that is distinct from the previous index.  Each index can only be used exactly once to
    /// remove a single item.
    /// 
    /// This method is an O(log(n)) operation.
    /// </summary>
    internal NativeHeapIndex Insert(in T t)
    {
        unsafe
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

            if (Data->Count == Data->Capacity)
            {
                Capacity *= 2;
            }

            var node = Data->Heap[Data->Count];
            node.Item = t;

            var insertIndex = Data->Count++;

            InsertAndBubbleUp(node, insertIndex);

            return new NativeHeapIndex()
            {
                TableIndex = node.TableIndex,
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                Version = Data->Table[node.TableIndex].Version,
                StructureId = Id
#endif
            };
        }
    }

    /// <summary>
    /// Removes the element tied to this NativeHeapIndex from the container.  The NativeHeapIndex must be
    /// the result of a previous call to Insert on this container.  If the item has already been removed by
    /// any means, this method will throw an ArgumentException.
    /// 
    /// This method will invalidate the provided index.  If you re-insert the removed object, you must use
    /// the NEW index to remove it again.
    /// 
    /// This method is an O(log(n)) operation.
    /// </summary>
    internal T Remove(NativeHeapIndex index)
    {
        unsafe
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);

            AssertValidIndex(index);
#endif
            int indexToRemove = Data->Table[index.TableIndex].HeapIndex;

            HeapNode<T> toRemove = Data->Heap[indexToRemove];

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            Data->Table[toRemove.TableIndex].Version++;
#endif

            HeapNode<T> lastNode = Data->Heap[--Data->Count];

            //First we move the node to remove to the end of the heap
            WriteArrayElement(Data->Heap, Data->Count, toRemove);

            if (indexToRemove != 0)
            {
                int parentIndex = (indexToRemove - 1) / 2;
                var parentNode = Data->Heap[parentIndex];
                if (Data->Comparator.Compare(lastNode.Item, parentNode.Item) < 0)
                {
                    InsertAndBubbleUp(lastNode, indexToRemove);
                    return toRemove.Item;
                }
            }

            //If we couldn't bubble up, bubbling down instead
            InsertAndBubbleDown(lastNode, indexToRemove);

            return toRemove.Item;
        }
    }

    /// <summary>
    /// Gets the element contained at this NativeHeapIndex.
    /// </summary>
    internal T Get(NativeHeapIndex index)
    {
        unsafe
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);

            AssertValidIndex(index);
#endif
            int indexInStorage = Data->Table[index.TableIndex].HeapIndex;
            HeapNode<T> node = Data->Heap[indexInStorage];
            return node.Item;
        }
    }

    #endregion

    #region IMPLEMENTATION

    private const int VALIDATION_ERROR_WRONG_INSTANCE = 1;
    private const int VALIDATION_ERROR_INVALID = 2;
    private const int VALIDATION_ERROR_REMOVED = 3;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
    private unsafe static int NextId = 1;
    private unsafe int Id;

    private unsafe AtomicSafetyHandle m_Safety;

    [NativeSetClassTypeToNullOnSchedule]
    private unsafe DisposeSentinel m_DisposeSentinel;
#endif

    [NativeDisableUnsafePtrRestriction]
    private unsafe HeapData<T, U>* Data;
    private unsafe Allocator Allocator;

    private unsafe NativeHeap(int initialCapacity, U comparator, Allocator allocator, int disposeSentinelStackDepth)
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        if (initialCapacity <= 0) {
            throw new ArgumentException(nameof(initialCapacity), "Must provide an initial capacity that is greater than zero.");
        }

        if (allocator == Allocator.None ||
            allocator == Allocator.Invalid ||
            allocator == Allocator.AudioKernel) {
            throw new ArgumentException(nameof(allocator), "Must provide an Allocator type of Temp, TempJob, or Persistent.");
        }

        DisposeSentinel.Create(out m_Safety, out m_DisposeSentinel, disposeSentinelStackDepth, allocator);
        Id = Interlocked.Increment(ref NextId);
#endif

        Data = (HeapData<T, U>*)Malloc(SizeOf<HeapData<T, U>>(), AlignOf<HeapData<T, U>>(), allocator);
        Data->Heap = (HeapNode<T>*)Malloc(SizeOf<HeapNode<T>>() * initialCapacity, AlignOf<HeapNode<T>>(), allocator);
        Data->Table = (TableValue*)Malloc(SizeOf<TableValue>() * initialCapacity, AlignOf<TableValue>(), allocator);

        Allocator = allocator;

        for (int i = 0; i < initialCapacity; i++)
        {
            Data->Heap[i] = new HeapNode<T>()
            {
                TableIndex = i
            };
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            Data->Table[i] = new TableValue() {
                Version = 1
            };
#endif
        }

        Data->Count = 0;
        Data->Capacity = initialCapacity;
        Data->Comparator = comparator;
    }

    private unsafe void InsertAndBubbleDown(HeapNode<T> node, int insertIndex)
    {
        while (true)
        {
            int indexL = insertIndex * 2 + 1;
            int indexR = insertIndex * 2 + 2;

            //If the left index is off the end, we are finished
            if (indexL >= Data->Count)
            {
                break;
            }

            if (indexR >= Data->Count || Data->Comparator.Compare(Data->Heap[indexL].Item,
                                                                  Data->Heap[indexR].Item) <= 0)
            {
                //left is smaller (or the only child)
                var leftNode = Data->Heap[indexL];

                if (Data->Comparator.Compare(node.Item, leftNode.Item) <= 0)
                {
                    //Last is smaller or equal to left, we are done
                    break;
                }

                Data->Heap[insertIndex] = leftNode;
                Data->Table[leftNode.TableIndex].HeapIndex = insertIndex;
                insertIndex = indexL;
            }
            else
            {
                //right is smaller
                var rightNode = Data->Heap[indexR];

                if (Data->Comparator.Compare(node.Item, rightNode.Item) <= 0)
                {
                    //Last is smaller than or equal to right, we are done
                    break;
                }

                Data->Heap[insertIndex] = rightNode;
                Data->Table[rightNode.TableIndex].HeapIndex = insertIndex;
                insertIndex = indexR;
            }
        }

        Data->Heap[insertIndex] = node;
        Data->Table[node.TableIndex].HeapIndex = insertIndex;
    }

    private unsafe void InsertAndBubbleUp(HeapNode<T> node, int insertIndex)
    {
        while (insertIndex != 0)
        {
            int parentIndex = (insertIndex - 1) / 2;
            var parentNode = Data->Heap[parentIndex];

            //If parent is actually less or equal to us, we are ok and can break out
            if (Data->Comparator.Compare(parentNode.Item, node.Item) <= 0)
            {
                break;
            }

            //We need to swap parent down
            Data->Heap[insertIndex] = parentNode;
            //Update table to point to new heap index
            Data->Table[parentNode.TableIndex].HeapIndex = insertIndex;

            //Restart loop trying to insert at parent index
            insertIndex = parentIndex;
        }

        Data->Heap[insertIndex] = node;
        Data->Table[node.TableIndex].HeapIndex = insertIndex;
    }

    [Conditional("ENABLE_UNITY_COLLECTIONS_CHECKS")]
    private unsafe void IsValidIndexInternal(NativeHeapIndex index, ref bool result, ref int errorCode)
    {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
        AtomicSafetyHandle.CheckReadAndThrow(m_Safety);

        if (index.StructureId != Id)
        {
            errorCode = VALIDATION_ERROR_WRONG_INSTANCE;
            result = false;
            return;
        }

        if (index.TableIndex >= Data->Capacity)
        {
            errorCode = VALIDATION_ERROR_INVALID;
            result = false;
            return;
        }

        TableValue tableValue = Data->Table[index.TableIndex];
        if (tableValue.Version != index.Version)
        {
            errorCode = VALIDATION_ERROR_REMOVED;
            result = false;
            return;
        }
#endif
    }

#endregion
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct TableValue
{
    internal int HeapIndex;
#if ENABLE_UNITY_COLLECTIONS_CHECKS
    internal int Version;
#endif
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct HeapData<T, U>
    where T : unmanaged
{
    internal int Count;
    internal int Capacity;
    internal unsafe HeapNode<T>* Heap;
    internal unsafe TableValue* Table;
    internal U Comparator;
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct HeapNode<T>
    where T : unmanaged
{
    internal T Item;
    internal int TableIndex;
}
