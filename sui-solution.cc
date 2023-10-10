

#include "search-strategies.h"
#include <vector>
#include "memusage.h"
#include <algorithm>
#include <unordered_map>
#include <iostream>
#include <deque>
#include <memory>
#include <set>

using namespace std;

struct StateBFS
{
	shared_ptr<SearchState> node;
	shared_ptr<StateBFS> prevNode;
	shared_ptr<SearchAction> actionFromPreviousState;
};

struct StateDFS
{
	shared_ptr<SearchState> node;
	shared_ptr<SearchState> prevNode;
	shared_ptr<SearchAction> actionFromPreviousState;
	uint32_t index;
};

vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state)
{
	cout << "=================" << endl;
	if (init_state.isFinal())
	{
		return {};
	}

	deque<shared_ptr<StateBFS>> open;
	unordered_map<shared_ptr<SearchState>, shared_ptr<StateBFS>> closed;

	// Initial state
	shared_ptr<StateBFS> initState = make_shared<StateBFS>();
	initState->node = make_shared<SearchState>(init_state);
	open.push_back(initState);

	bool found = false;
	// Cycle through the tree
	while (!open.empty() && found == false)
	{
		shared_ptr<StateBFS> currentState = move(open.back());
		open.pop_back();

		// Check current state for final
		if (currentState->node->isFinal())
		{
			open.push_front(move(currentState));
			break;
		}

		// closed[currentState->node] = currentState;
		closed[currentState->node] = currentState;

		// Save all child-nodes to open
		for (auto &action : currentState->node->actions())
		{
			if (getCurrentRSS() + 1000000 > mem_limit_)
			{
				cout << "Dosla pamet" << endl;
				cout << getCurrentRSS() <<endl;
				cout << open.size() << "  " << closed.size() << endl;
				return {};
			}

			shared_ptr<StateBFS> nextState = make_shared<StateBFS>();
			nextState->node = make_shared<SearchState>(action.execute(*currentState->node));
			nextState->actionFromPreviousState = make_shared<SearchAction>(action);
			nextState->prevNode = currentState;

			if (nextState->node->isFinal())
			{
				found = true;
				open.push_front(move(nextState));
				break;
			}
			// Insert only not visited nodes
			else if (closed.find(nextState->node) == closed.end())
			{
				open.push_front(move(nextState));
			}
		}
	}

	cout << open.size() << "  " << closed.size() << endl;


	// Create path to final node
	vector<SearchAction> solution = {};
	if (!closed.empty() && open.front()->node->isFinal())
	{
		shared_ptr<StateBFS> actualState = open.front();
		while (actualState->node != initState->node)
		{
			solution.push_back(*(actualState->actionFromPreviousState));
			auto prevState = actualState->prevNode;
			actualState = prevState;
		}
		reverse(solution.begin(), solution.end());
		return solution;
	}

	return {};
}

vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state)
{
	if (init_state.isFinal())
	{
		return {};
	}

	deque<shared_ptr<StateDFS>> open;
	unordered_map<shared_ptr<SearchState>, shared_ptr<StateDFS>> closed;

	// Initial state
	shared_ptr<StateDFS> initState = make_shared<StateDFS>();
	initState->node = make_shared<SearchState>(init_state);
	initState->index = 0;
	open.push_back(initState);

	// Cycle through the tree
	while (!open.empty())
	{
		shared_ptr<StateDFS> currentState = move(open.back());
		open.pop_back();

		if (currentState->index < depth_limit_)
		{
			// Check current state for final
			if (currentState->node->isFinal())
			{
				open.push_back(move(currentState));
				break;
			}

			closed[currentState->node] = currentState;

			// Save all child-nodes to open
			for (auto action : currentState->node->actions())
			{
				if (getCurrentRSS() > (mem_limit_ * 0.97))
				{
					return {};
				}

				shared_ptr<StateDFS> nextState = make_shared<StateDFS>();
				nextState->node = make_shared<SearchState>(action.execute(*currentState->node));

				// Insert only not visited nodes
				if (closed.find(nextState->node) == closed.end())
				{
					nextState->actionFromPreviousState = make_shared<SearchAction>(action);
					nextState->prevNode = currentState->node;
					nextState->index = currentState->index + 1;
					open.push_back(move(nextState));
				}
			}
		}
		else
		{
			open.pop_back();
		}
	}

	// Create path to final node
	vector<SearchAction> solution = {};
	if (!closed.empty() && !open.empty() && open.back()->node->isFinal())
	{
		shared_ptr<StateDFS> actualState = open.back();
		while (actualState->node != initState->node)
		{
			solution.push_back(*(actualState->actionFromPreviousState));
			shared_ptr<StateDFS> prevState = make_shared<StateDFS>();
			prevState->node = actualState->prevNode;

			auto it = closed.find(prevState->node);
			if (it != closed.end())
			{
				actualState = it->second; // Assign actualState to the found element
			}
		}
		reverse(solution.begin(), solution.end());
		return solution;
	}

	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const
{
	return 0;
}

vector<SearchAction> AStarSearch::solve(const SearchState &init_state)
{
	return {};
}
