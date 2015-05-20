BUILDDIR=build/


fmrchallenge: fmrchallenge.tex
	pdflatex -output-directory $(BUILDDIR) $<
	bibtex $(BUILDDIR)$@
	pdflatex -output-directory $(BUILDDIR) $<
	pdflatex -output-directory $(BUILDDIR) $<

all: fmrchallenge icra2015formalmethodswrkshp

icra2015formalmethodswrkshp: icra2015formalmethodswrkshp.tex
	pdflatex -output-directory $(BUILDDIR) $<

date: fmrchallenge
	cp $(BUILDDIR)$<.pdf $(BUILDDIR)$<-DRAFT-`date -u "+%Y%m%d-%H%M"`.pdf

gitdate: fmrchallenge
	cp $(BUILDDIR)$<.pdf $(BUILDDIR)$<-`git log -1 --format=%H | cut -c 1-7`-DRAFT-`date -u "+%Y%m%d-%H%M"`.pdf

clean:
	rm -f *~ $(BUILDDIR)*
