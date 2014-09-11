BUILDDIR=build/

all: icra2015formalmethodschallenge

icra2015formalmethodschallenge: icra2015formalmethodschallenge.tex
	pdflatex -output-directory $(BUILDDIR) $<
	pdflatex -output-directory $(BUILDDIR) $<

date: icra2015formalmethodschallenge
	cp $(BUILDDIR)$<{,-DRAFT-`date "+%Y%m%d-%H%M"`}.pdf

clean:
	rm -f *~ $(BUILDDIR)*
