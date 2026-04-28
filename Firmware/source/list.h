#ifndef LIST_H
#define LIST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

	typedef struct _tListNode {
		struct _tListNode* next;
		struct _tListNode* prev;
	}tListNode, * pListNode;


	void list_init_head(tListNode* listHead);

	bool list_is_empty(tListNode* listHead);

	void list_insert_head(tListNode* listHead, tListNode* node);

	void list_insert_tail(tListNode* listHead, tListNode* node);

	void list_remove_node(tListNode* node);

	void list_remove_head(tListNode* listHead, tListNode** node);

	void list_remove_tail(tListNode* listHead, tListNode** node);

	void list_insert_node_after(tListNode* node, tListNode* ref_node);

	void list_insert_node_before(tListNode* node, tListNode* ref_node);

	int list_get_size(tListNode* listHead);

	void list_get_next_node(tListNode* ref_node, tListNode** node);

	void list_get_prev_node(tListNode* ref_node, tListNode** node);

#ifdef __cplusplus
}
#endif

#endif	/* LIST_H */

