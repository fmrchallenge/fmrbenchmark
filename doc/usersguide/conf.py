# -*- coding: utf-8 -*-

extensions = ['sphinx.ext.mathjax']
templates_path = ['_templates']
pygments_style = 'sphinx'
exclude_patterns = ['_build']
source_suffix = '.rst'
master_doc = 'index'

project = u'fmrbenchmark'
copyright = u'2015-2016, Caltech et al.'

version = '0.0.4'
release = version  # To appease Sphinx


html_theme = 'default'
html_theme_options = {'collapsiblesidebar': True}
html_title = project+' '+version
html_static_path = ['_static']
html_extra_path = []
html_last_updated_fmt = '%d %b %Y'
