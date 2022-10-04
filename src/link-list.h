/*
 * link-list.h
 *
 *  Created on: Sep 30, 2021
 *      Author: etaash
 */

#ifndef LINK_LIST_H_
#define LINK_LIST_H_

//#include <array>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <inttypes.h>
#include <optional>


//TODO: FIX NULL BASENODE!!!!!!
#define REPEAT(i, size) for(__typeof__(size) i = 0; i < size; i++)

template <typename T>
struct Node{
public:
	T value;
	Node<T>* next;
	Node<T>* prev;
	Node(T value, Node<T>* next, Node<T>* prev){
		this->value = value;
		this->next = next;
		this->prev = prev;
	}

	Node<T>& operator--(){
		return *this->prev;
	}

	Node<T> operator--(int){
		Node<T> old = *this;
		old = old.operator--();
		return old;
	}

	Node<T> operator++(int){
		Node<T> old = *this;
		old = old.operator++();
		return old;
	}

	Node<T>& operator++(){
		return *this->next;
	}

	//get the value using T& value = (T&)node;
	explicit operator T&(){
		return value;
	}
};

template <typename T>
class LinkedList{
public:

	LinkedList(){
		basenode = nullptr;
		numnodes = 0;
		tail = basenode;
	}

	LinkedList(T value){
		basenode = new Node<T>(value, nullptr, nullptr);
		numnodes = 1;
		tail = basenode;
	}

	LinkedList(T* values, size_t length){
		basenode = new Node<T>(values[0],nullptr, nullptr);
		numnodes = 1;
		Node<T>* prev_init = &basenode;
		for(size_t i = 1; i < length; i++){
			Node<T>* temp = new Node<T>(values[i],nullptr,prev_init);
			prev_init->next=temp;
			prev_init = temp;
			tail = temp;
			numnodes++;
		}
	}

	Node<T>& push_back(T value){ //bruh
		if(size() != 0){
			Node<T>* temp_tail = new Node<T>(value, tail, nullptr);
			tail->next = temp_tail;
			tail = temp_tail;
		}
		else{
			tail = new Node<T>(value, nullptr, nullptr);
			basenode = tail;
		}
		numnodes++;
		return *tail;
	}

	std::optional<Node<T>> pop_back(){
		if(tail->prev != nullptr){
			Node<T>* prev_tail = tail;
			tail->prev->next = nullptr;
			delete tail;
			tail = tail->prev;
			numnodes--;
			return prev_tail;
		}
		return {};
	}


	Node<T>& operator[](size_t index){
		assert(!(index >= size()));
		Node<T>* Temp = basenode;
		REPEAT(i, index){
			Temp = Temp->next;
		}
		return *Temp;
	}

	size_t size(){
		return numnodes;
	}

	std::optional<Node<T>> pop_front(){
		if(basenode){
			Node<T>* prev_basenode = basenode;
			Node<T> _prev_basenode = *prev_basenode;
			basenode = basenode->next;
			delete prev_basenode;
			numnodes--;
			return _prev_basenode;
		}
		return {};
	}

	Node<T>& push_front(T value){
		basenode = new Node<T>(value, basenode, nullptr);
		numnodes++;
		return *basenode;
	}


	Node<T>* basenode;
	Node<T>* tail;

private:

	size_t numnodes;
};

template<typename T>
class Stack{
public:
	Stack(){
	}

	void Push(T value){
		stack.push_front(value);
	}

	void Pop(){
		stack.pop_front();
	}

	size_t Size(){
		return stack.size();
	}

	Node<T>& get_node(size_t index){
		return stack[index];
	}

	T& operator[](size_t index){
		return (T&)stack[index];
	}
private:
	LinkedList<T> stack;
};

template<typename T>
class Queue{
public:
	Queue(){}

	void Push(T value){
		queue.push_back(value);
	}

	void Pop(){
		queue.pop_back();
	}
	
	size_t Size(){
		return queue.size();
	}

	Node<T>& get_node(size_t index){
		return queue[index];
	}

	T& operator[](size_t index){
		return (T&)queue[index];
	}
private:
	LinkedList<T> queue;
};

#endif /* LINK_LIST_H_ */
