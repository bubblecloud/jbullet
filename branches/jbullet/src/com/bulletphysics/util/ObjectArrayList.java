/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.util;

import java.util.AbstractList;
import java.util.RandomAccess;

/**
 *
 * @author jezek2
 */
public class ObjectArrayList<T> extends AbstractList<T> implements RandomAccess {

	private T[] array;
	private int size;

	public ObjectArrayList() {
		this(16);
	}
	
	@SuppressWarnings("unchecked")
	public ObjectArrayList(int initialCapacity) {
		array = (T[])new Object[initialCapacity];
	}
	
	@Override
	public boolean add(T value) {
		if (size == array.length) {
			expand();
		}
		
		array[size++] = value;
		return true;
	}
	
	@SuppressWarnings("unchecked")
	private void expand() {
		T[] newArray = (T[])new Object[array.length << 1];
		System.arraycopy(array, 0, newArray, 0, array.length);
		array = newArray;
	}

	@Override
	public T remove(int index) {
		if (index >= size) throw new IndexOutOfBoundsException();
		T old = array[index];
		System.arraycopy(array, index+1, array, index, size - index - 1);
		size--;
		return old;
	}

	public T get(int index) {
		if (index >= size) throw new IndexOutOfBoundsException();
		return array[index];
	}

	@Override
	public T set(int index, T value) {
		if (index >= size) throw new IndexOutOfBoundsException();
		T old = array[index];
		array[index] = value;
		return old;
	}

	public int size() {
		return size;
	}
	
	public int capacity() {
		return array.length;
	}
	
	@Override
	public void clear() {
		size = 0;
	}

	@Override
	public int indexOf(Object o) {
		for (int i=0; i<size; i++) {
			if (o == null? array[i] == null : o.equals(array[i])) {
				return i;
			}
		}
		return -1;
	}
	
}
