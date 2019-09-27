#include "IO/Poll.h"

#include <err.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/epoll.h>
#include <unistd.h>

#include <vector>

namespace IO
{

Pollable::~Pollable()
{
}

PollGroup::PollGroup()
{
	m_fd = epoll_create(10); // note: 10 is just a hint!
	if (m_fd < 0)
		err(EXIT_FAILURE, "PollGroup: epoll_create failed");
}

PollGroup::~PollGroup()
{
	close(m_fd);
}

void PollGroup::add(int fd, std::function<void()> handler)
{
	if (m_handlers.emplace(fd, handler).second == false)
		errx(EXIT_FAILURE, "PollGroup: add called on with an already-inserted fd");

	epoll_event ev;
	ev.events = EPOLLIN;
	ev.data.fd = fd;
	epoll_ctl(m_fd, EPOLL_CTL_ADD, fd, &ev);
}

void PollGroup::remove(int fd)
{
	if (m_handlers.erase(fd) != 1)
		errx(EXIT_FAILURE, "PollGroup: remove called with a non-present fd");

	epoll_event dummy;
	epoll_ctl(m_fd, EPOLL_CTL_DEL, fd, &dummy);
}

void PollGroup::add(Pollable *object)
{
	add(object->fd(), std::bind(&Pollable::runOnce, object));
}

void PollGroup::remove(Pollable *object)
{
	remove(object->fd());
}

int PollGroup::fd() const
{
	return m_fd;
}

void PollGroup::runOnce()
{
	epoll_event evt;
restart:
	int nfds = epoll_wait(m_fd, &evt, 1, -1);
	if (nfds != 1)
	{
		if (errno == EINTR)
			goto restart;

		err(EXIT_FAILURE, "PollGroup: epoll_wait");
	}

	int fd = evt.data.fd;
	std::map<int, std::function<void()>>::const_iterator it = m_handlers.find(fd);

	if (it != m_handlers.cend() && it->second)
	{
		std::function<void()> handler = it->second;
		handler();
	}
	else
	{
		errx(EXIT_FAILURE, "PollGroup: activity on a fd without a registered handler");
	}
}

}
