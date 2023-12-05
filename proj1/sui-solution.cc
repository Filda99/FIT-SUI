#include "search-strategies.h"
#include <vector>
#include "memusage.h"
#include <algorithm>
#include <unordered_set>
#include <iostream>
#include <deque>
#include <memory>
#include <set>
#include <queue>

#define SPACE_RESERVED 50000000


/*************************************************************
 * STRUCTURES *
 *************************************************************/

struct StateBFS
{
	std::shared_ptr<SearchState> node;
	std::shared_ptr<StateBFS> prevNode;
	std::shared_ptr<SearchAction> actionFromPreviousState;
};

struct StateDFS
{
	std::shared_ptr<SearchState> node;
	std::shared_ptr<StateDFS> prevNode;
	std::shared_ptr<SearchAction> actionFromPreviousState;
	int index;
};

struct StateAStar
{
	std::shared_ptr<SearchState> node;
	std::shared_ptr<StateAStar> prevNode;
	std::shared_ptr<SearchAction> actionFromPreviousState;
	double priority;
	int index;
};

struct StateAStarCompare
{
	bool operator()(const std::shared_ptr<StateAStar> &lhs, const std::shared_ptr<StateAStar> &rhs) const
	{
		return lhs->priority > rhs->priority;
	}
};

/*************************************************************
 * HASH FUNCTIONS *
 *************************************************************/

template <typename StateType>
struct StateEquality
{
	bool operator()(const std::shared_ptr<StateType> &lhs,
					const std::shared_ptr<StateType> &rhs) const
	{
		return *lhs->node == *rhs->node;
	}
};

template <typename StateType>
struct StateHash
{
	size_t operator()(const std::shared_ptr<StateType> &state) const
	{
		return hash(*state->node);
	}
};

bool operator==(const SearchState &a, const SearchState &b)
{
	return a.state_ == b.state_;
}

inline size_t hash_card(const Card &card)
{
	return std::hash<Color>()(card.color) ^
		   (std::hash<int>()(card.value) << 1);
}

size_t hash(const SearchState &state)
{
	size_t hash = 0;

	for (const auto &freeCell : state.state_.free_cells)
	{
		auto card = freeCell.topCard();
		if (card.has_value())
		{
			hash ^= hash_card(card.value());
		}
	}

	for (const auto &tableauStack : state.state_.stacks)
	{
		for (const auto &card : tableauStack.storage())
		{
			hash ^= hash_card(card) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		}
	}

	return hash;
}

/*************************************************************
 * BFS *
 *************************************************************/

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state)
{
	if (init_state.isFinal())
	{
		return {};
	}

	std::deque<std::shared_ptr<StateBFS>> open;
	std::unordered_set<std::shared_ptr<StateBFS>, StateHash<StateBFS>, StateEquality<StateBFS>> closed;

	// Initial state
	std::shared_ptr<StateBFS> initState = std::make_shared<StateBFS>();
	initState->node = std::make_shared<SearchState>(init_state);
	open.push_back(initState);

	bool found = false;
	// Cycle through the tree
	while (!open.empty() && !found)
	{
		std::shared_ptr<StateBFS> currentState = std::move(open.back());
		open.pop_back();

		if (closed.find(currentState) != closed.end())
		{
			continue;
		}

		closed.insert(currentState);

		// Save all child-nodes to open
		for (auto &action : currentState->node->actions())
		{
			if (getCurrentRSS() + SPACE_RESERVED > mem_limit_)
			{
				return {};
			}

			std::shared_ptr<StateBFS> nextState = std::make_shared<StateBFS>();
			nextState->node = std::make_shared<SearchState>(action.execute(*currentState->node));
			nextState->actionFromPreviousState = std::make_shared<SearchAction>(action);
			nextState->prevNode = currentState;

			if (nextState->node->isFinal())
			{
				found = true;
				open.push_front(std::move(nextState));
				break;
			}
			// Insert only not visited nodes
			else if (closed.find(nextState) == closed.end())
			{
				open.push_front(std::move(nextState));
			}
		}
	}

	// Create path to final node
	std::vector<SearchAction> solution = {};
	if (!closed.empty() && open.front()->node->isFinal())
	{
		std::shared_ptr<StateBFS> actualState = open.front();
		while (actualState->node != initState->node)
		{
			solution.push_back(*(actualState->actionFromPreviousState));
			actualState = actualState->prevNode;
		}
		reverse(solution.begin(), solution.end());
		return solution;
	}

	return {};
}

/*************************************************************
 * DFS *
 *************************************************************/

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state)
{
	if (init_state.isFinal())
	{
		return {};
	}

	std::deque<std::shared_ptr<StateDFS>> open;

	// Initial state
	std::shared_ptr<StateDFS> initState = std::make_shared<StateDFS>();
	initState->node = std::make_shared<SearchState>(init_state);
	initState->index = 0;
	open.push_back(initState);

	bool found = false;
	// Cycle through the tree
	while (!open.empty() && !found)
	{
		std::shared_ptr<StateDFS> currentState = std::move(open.back());
		open.pop_back();

		if (currentState->index < depth_limit_)
		{
			// Save all child-nodes to open
			for (auto action : currentState->node->actions())
			{
				if (getCurrentRSS() + SPACE_RESERVED > mem_limit_)
				{
					return {};
				}

				std::shared_ptr<StateDFS> nextState = std::make_shared<StateDFS>();
				nextState->node = std::make_shared<SearchState>(action.execute(*currentState->node));
				nextState->actionFromPreviousState = std::make_shared<SearchAction>(action);
				nextState->prevNode = currentState;
				nextState->index = currentState->index + 1;

				if (nextState->node->isFinal())
				{
					found = true;
					open.push_front(std::move(nextState));
					break;
				}
				open.push_back(std::move(nextState));
			}
		}
		else
		{
			open.pop_back();
		}
	}

	// Create path to final node
	std::vector<SearchAction> solution = {};
	if (!open.empty() && open.front()->node->isFinal())
	{
		std::shared_ptr<StateDFS> actualState = move(open.front());
		while (actualState->node != initState->node)
		{
			solution.push_back(*(actualState->actionFromPreviousState));
			actualState = actualState->prevNode;
		}
		reverse(solution.begin(), solution.end());
		return solution;
	}

	return {};
}

/*************************************************************
 * A STAR *
 *************************************************************/

// source: https://www.johnkoza.com/gp.org/hc2013/Sipper-Paper.pdf
// Cards out of a order
double NotInOder(const GameState &state) {
    double CardsNotInOrder = 0;
    std::vector<int> allCardValuesFromAStack; // Create an array to collect card values
    // Iterate through cascade (tableau) piles
    for (const auto &tableauStack : state.stacks) {
        auto stack = tableauStack.storage();
        for (const auto &card : stack) {
            allCardValuesFromAStack.push_back(card.value); // Collect all card values
        }
    }
    // Count occurrences of numbers that are not in order
    for (size_t i = 0; i < allCardValuesFromAStack.size() - 1; i++) {
        if (allCardValuesFromAStack[i] > allCardValuesFromAStack[i + 1]) {
            CardsNotInOrder += 1.0; // Increment for each out-of-order card
        }
    }

    // Normalize the cards out of order heuristic
    return (CardsNotInOrder/52); //52 == maximum of cards out of homes
} 

// Count the number of cards that are not at the foundation piles
double NumCardsNotAtFoundations(const GameState &state) {
    double count = 0;
    const auto &firstCard = state.stacks[0].storage().back();
    for (const auto &free_cells : state.free_cells) {
        if (!free_cells.canAccept(firstCard)) //how many cards there are
            count++;
    }
    for (const auto &stack : state.stacks) {
        count += stack.storage().size();
    }
    return count;
} 

double StudentHeuristic::distanceLowerBound(const GameState &state) const
{
	double heuristic = 0.0;
	heuristic += NumCardsNotAtFoundations(state);
	heuristic += NotInOder(state);
	return floor(heuristic / 2);
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state)
{
	if (init_state.isFinal())
	{
		return {};
	}

	std::priority_queue<std::shared_ptr<StateAStar>,
						std::vector<std::shared_ptr<StateAStar>>,
						StateAStarCompare>
		openPrio;

	std::unordered_set<std::shared_ptr<StateAStar>,
					   StateHash<StateAStar>,
					   StateEquality<StateAStar>>
		closed;

	// Initial state
	std::shared_ptr<StateAStar> initState = std::make_shared<StateAStar>();
	initState->node = std::make_shared<SearchState>(init_state);
	initState->index = 0;
	openPrio.push(initState);

	bool found = false;
	// Cycle through the tree
	while (!openPrio.empty() && !found)
	{
		std::shared_ptr<StateAStar> currentState = std::move(openPrio.top());
		openPrio.pop();

		if (closed.find(currentState) != closed.end())
		{
			continue;
		}

		closed.insert(currentState);

		// Save all child-nodes to openPrio
		for (auto &action : currentState->node->actions())
		{
			if (getCurrentRSS() + SPACE_RESERVED > mem_limit_)
			{
				return {};
			}

			std::shared_ptr<StateAStar> nextState = std::make_shared<StateAStar>();
			nextState->node = std::make_shared<SearchState>(action.execute(*currentState->node));
			nextState->actionFromPreviousState = std::make_shared<SearchAction>(action);
			nextState->prevNode = currentState;

			if (nextState->node->isFinal())
			{
				found = true;
				openPrio.push(std::move(nextState));
				break;
			}
			// Insert only not visited nodes
			else if (closed.find(nextState) == closed.end())
			{
				auto heuristic = compute_heuristic(*nextState->node, *heuristic_);
				nextState->index = currentState->index + 1;
				nextState->priority = heuristic + nextState->index;
				openPrio.push(std::move(nextState));
			}
		}
	}

	// Create path to final node
	std::vector<SearchAction> solution = {};
	if (!closed.empty() && openPrio.top()->node->isFinal())
	{
		std::shared_ptr<StateAStar> actualState = openPrio.top();
		while (actualState->node != initState->node)
		{
			solution.push_back(*(actualState->actionFromPreviousState));
			actualState = actualState->prevNode;
		}
		reverse(solution.begin(), solution.end());
		return solution;
	}

	return {};
}
