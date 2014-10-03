BUILDDIR=build/


icra2015formalmethodschallenge: icra2015formalmethodschallenge.tex
	pdflatex -output-directory $(BUILDDIR) $<
	bibtex $(BUILDDIR)$@
	pdflatex -output-directory $(BUILDDIR) $<
	pdflatex -output-directory $(BUILDDIR) $<

all: icra2015formalmethodschallenge icra2015formalmethodswrkshp

icra2015formalmethodswrkshp: icra2015formalmethodswrkshp.tex
	pdflatex -output-directory $(BUILDDIR) $<

date: icra2015formalmethodschallenge
	cp $(BUILDDIR)$<{,-DRAFT-`date -u "+%Y%m%d-%H%M"`}.pdf

clean:
	rm -f *~ $(BUILDDIR)*
