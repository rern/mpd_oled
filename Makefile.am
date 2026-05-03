ACLOCAL_AMFLAGS = -I m4

SUBDIRS = src

filesdir = ${datarootdir}/${PACKAGE}
format_all:
	for f in src/*.cpp src/*.h ; do \
	clang-format -style=file -i $$f; \
	done
