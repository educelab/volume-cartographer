// CLinkedList.h
// Chao Du 2015 Jan
#ifndef _CLINKEDLIST_H_
#define _CLINKEDLIST_H_

#include "HBase.h"

namespace ChaoVis {

template < typename T >
class CLinkedList {
	
public: // REVISIT - probably we can hide the node class
	// internal node class
    template < typename S >
	class CLinkedListNode {

	public:
		CLinkedListNode( void ) : fPrev( NULL ), fNext( NULL ), fData( NULL ) { }
        CLinkedListNode( const S *nData ) : fPrev( NULL ), fNext( NULL ), fData( NULL ) {
            fData = new S;
			*fData = *nData;
		}

		~CLinkedListNode( void ) {
			deleteNULL( fData );
		}

        void SetData( const S *nData ) {
			if ( fData == NULL ) {
                fData = new S;
			}
			*fData = *nData;
		}
        S GetData( void ) {
			return *fData;
		}

		CLinkedListNode* GetNext( void ) { return fNext; }
		CLinkedListNode* GetPrev( void ) { return fPrev; }
		void SetNext( CLinkedListNode *nNext ) { fNext = nNext; }
		void SetPrev( CLinkedListNode *nPrev ) { fPrev = nPrev; }


	private:
        S *fData;
		CLinkedListNode *fPrev;	// REVISIT - an alternative easier way is to expose these pointers
		CLinkedListNode *fNext;
	}; // class CLinkedListNode

public:
	CLinkedList( void ) : fHead( NULL ), fTail( NULL ), fNodeNum( 0 ) { }
	~CLinkedList( void ) {
		Clear();
	}

	void Append( const T *nData ) {
		// create a new node
		CLinkedListNode< T > *aNewNode = new CLinkedListNode< T >( nData );
		
		if ( fNodeNum == 0 ) {
			fHead = aNewNode;
			fTail = aNewNode;
		} else {
			// append to the tail
			fTail->SetNext( aNewNode );
			aNewNode->SetPrev( fTail );
		
			// update tail
			fTail = aNewNode;
		}
		// update the number of nodes
		++fNodeNum;
	}
	void RevAppend( const T *nData ) {
		// create a new node
		CLinkedListNode< T > *aNewNode = new CLinkedListNode< T >( nData );

		if ( fNodeNum == 0 ) {
			fHead = aNewNode;
			fTail = aNewNode;
		} else {
			// reverse append to the head
			fHead->SetPrev( aNewNode );
			aNewNode->SetNext( fHead );

			// update head
			fHead = aNewNode;
		}

		// update the number of nodes
		++fNodeNum;
	}
	
	void Remove( void ) {
		if ( fTail != fHead ) { // # of elements > 1
			// remove the node at the tail
			CLinkedListNode< T > *aTmpPtr = fTail;
			fTail = fTail->GetPrev();
			fTail->SetNext( NULL ) ;

			delete aTmpPtr;

			--fNodeNum;
		} else {
			if ( fTail != NULL ) { // # of elements = 1
				delete fTail;
				fTail = NULL;
				fHead = NULL;
				--fNodeNum;
			}
		}
	}
	void RevRemove( void ) {
		if ( fHead != fTail ) { // # of elements > 1
			// remove the node at the head
			CLinkedListNode< T > *aTmpPtr = fHead;
			fHead = fHead->GetNext();
			fHead->SetPrev( NULL );

            delete aTmpPtr;

			--fNodeNum;
		} else { // # of elements = 1
			if ( fHead != NULL ) {
				delete fHead;
				fHead = NULL;
				fTail = NULL;
				--fNodeNum;
			}
		}
	}

	void Clear( void ) {
		CLinkedListNode< T > *aTmpPtr;
		while ( fHead != NULL ) {
			aTmpPtr = fHead;
			fHead = fHead->GetNext();
			delete( aTmpPtr );	// REVISIT - BUGGY - double check to make sure the destructor is invoked
		}
	}

	int GetSize( void ) { return fNodeNum; }

	// Linearly traverse the list and return the ith item
	T GetData( int nIndex ) const {
		int aCount = 0;
		CLinkedListNode< T > *aPtr = fHead;
		while ( aCount != nIndex ) {
			++aCount;
			aPtr = aPtr->GetNext();
		}
		return aPtr->GetData();
	}

protected:

private:
	CLinkedListNode< T > *fHead;
	CLinkedListNode< T > *fTail;
	int fNodeNum;

}; // class CLinkedList

} // namespace ChaoVis

#endif // _CLINKEDLIST_H_
