#ifndef IO_POLL_H
#define IO_POLL_H

#include <functional>
#include <map>

namespace IO
{

// Inferface for an object that PollGroup can wait for
class Pollable
{
	public:
		virtual ~Pollable();

		// the fd must stay constant for the whole lifetime of the object
		virtual int fd() const = 0;
		virtual void runOnce() = 0;
};

/* Class that wraps Linux's epoll(7) interface. It can be used to wait until
 * at least one file descriptor in a group becomes ready to be read.
 *
 * It can be used in three ways:
 *
 *  1) Call add() with a Pollable object. Then, when runOnce() is called, if
 *     there is data ready to be read on the file descriptor returned by the
 *     object's fd() method, that object's runOnce() method will be called.
 *
 *  2) Call add() with a file descriptor and valid "handler" function. Then,
 *     when runOnce() is called, if there is data ready to be read on that file
 *     descriptor, that function will be called.
 *
 *  3) Call add() with a file descriptor but no handler. In this mode, runOnce()
 *     must NEVER be called, but fd() can still be used to wait for activity on
 *     any registered file descriptor (see the following paragraph).
 *
 * If no registered objects or file descriptors are ready, runOnce() will block
 * until at least one becomes ready. It also possible to wait until at least one
 * file descriptor is ready by poll()ing/select()ing (or even adding to a
 * different PollManager instance) the file descriptor returned by fd().
 */
class PollGroup : public Pollable
{
	public:
		PollGroup();
		~PollGroup() override;

		// add/remove raw watched file descriptors
		// if handler is nullptr, fd() can still be used, but runOnce()
		// must never be called
		void add(int fd, std::function<void()> handler = nullptr);
		void remove(int fd);

		// add/remove Pollable objects
		void add(Pollable *object);
		void remove(Pollable *object);

		// this fd can be used to poll on any registered fd
		int fd() const override;

		// poll and run one active fd handler
		void runOnce() override;

	private:
		std::map<int, std::function<void()>> m_handlers;
		int m_fd; // epoll fd
};

}

#endif // IO_POLL_H
