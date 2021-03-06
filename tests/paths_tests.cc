/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <catch.hpp>

#include "paths.hh"

using namespace std;
using namespace ftl;

namespace Catch {
    template<> struct StringMaker<Path> {
        static std::string convert(const Path &value) {
            return value.stringValue();
        }
    }; 
}

TEST_CASE("path matching") {
	GIVEN("a prefix pattern") {
		PathPattern pp({ "/foo", "/bar" });

		REQUIRE(pp.match("/foo/bar.c"));
		REQUIRE(pp.match("/bar/foo.c"));
		REQUIRE(!pp.match("/other/foo.c"));
		
		REQUIRE(!pp.match("/other/foo.c"));
	}
}

TEST_CASE("path") {
	GIVEN("two equal paths") {
		Path p1("/foo/bar/");
		Path p2("/foo/bar");

		THEN("equality check should perform normalization") {
			REQUIRE(p1 == p2);
		}
	}

	
	GIVEN("a non-normalized path") {
		Path p("/./foo/bar/../baz/");
		
		REQUIRE(!p.inNormalForm());
		REQUIRE(p.size() == 5);

		THEN("split should return the expected components") {
			const auto split = p.split(false);
			REQUIRE(split == (vector<string> { "/", ".", "foo", "bar", "..", "baz" }));
		}

		THEN("normalization should drop '.', '..', '//', and trailing '/") {
			const auto split = p.split(true);
			REQUIRE(split == (vector<string> { "/", "foo", "baz" }));
		}
	}

	GIVEN("a normalized path") {
		Path p("/foo/bar");

		REQUIRE(p.inNormalForm());
		REQUIRE(p.size() == 2);

		THEN("split should return the expected components") {
			const auto split = p.split(false);
			REQUIRE(split == (vector<string> { "/", "foo", "bar" }));
		}
		
		THEN("basename should return the last component") {
			const auto rel = p.basename();
			REQUIRE(rel == Path("bar"));
		}
	}

	GIVEN("a set of child and parent paths") {
		Path root("/foo");
		Path parent("/foo/bar/");
		Path file("/foo/bar/foo.c");
		Path sub1("/foo/bar/sub1");
		Path nnFile("/foo/bar/sub1/sub2/../../foo.c");
		
		WHEN("performing prefix comparison") {
			THEN("children should be normalized before compare") {
				REQUIRE(!nnFile.hasPrefix(sub1));
				REQUIRE(nnFile.hasPrefix(parent));
			}
			
		}

		WHEN("performing prefix trimming") {			
			THEN("a non-matching prefix should return None") {
				auto t = file.trimPrefix(Path("/notprefix"), true);
				REQUIRE(t.is<Nothing>());
			}

			THEN("trimming an identical path should return None") {
				auto t = file.trimPrefix(file, true);
				REQUIRE(t.is<Nothing>());
			}

			THEN("trimming a child should succeed") {
				auto tr = file.trimPrefix(root, true);
				auto ta = file.trimPrefix(root, false);
	
				REQUIRE(tr == just(Path("bar/foo.c")));
				REQUIRE(ta == just(Path("/bar/foo.c")));
			}
		}
	}
}
