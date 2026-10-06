#include <cassert>

#include "scandium/network/lockstep.hpp"

int main() {
    using namespace scandium::network;

    LockstepSession session{{10, 20}};
    session.submit(10, {7, 1, 0});
    assert(!session.ready(7));

    session.submit(20, {7, 0, 1});
    assert(session.ready(7));

    const auto frames = session.consume(7);
    assert(frames.has_value());
    assert(frames->size() == 2);
    assert((*frames)[0].axis_x == 1);
    assert((*frames)[1].axis_y == 1);
    assert(!session.ready(7));

    session.submit(999, {8, 4, 4});
    assert(!session.ready(8));

    return 0;
}
