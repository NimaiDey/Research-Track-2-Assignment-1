# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#

import os
import sys
sys.path.insert(0, os.path.abspath('../../scripts'))
sys.path.insert(0, os.path.abspath('/home/nimailinux/catkin_ws/devel/lib/python3/dist-packages'))

# -- Project information -----------------------------------------------------
project = 'assignment_2_2022-main'
copyright = '2025, NimaiDey'
author = 'NimaiDey'
version = ''
release = 'RT 1.0'

# -- General configuration ---------------------------------------------------
extensions = [
	'sphinx.ext.autodoc',
	'sphinx.ext.doctest',
	'sphinx.ext.intersphinx',
	'sphinx.ext.todo',
	'sphinx.ext.coverage',
	'sphinx.ext.mathjax',
	'sphinx.ext.ifconfig',
	'sphinx.ext.viewcode',
	'sphinx.ext.githubpages',
	"sphinx.ext.napoleon",
	'sphinx.ext.inheritance_diagram'
]

templates_path = ['_templates']
source_suffix = '.rst'
master_doc = 'index'
language = None
exclude_patterns = []
pygments_style = None

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # Use Sphinx Read the Docs theme
html_theme_options = {
    'logo_only': False,  # Do not use logo in the header
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    'vcs_pageview_mode': 'blob',
    'style_nav_header_background': '#2e3b4e',  # Dark background for header
    'collapse_navigation': False,
    'sticky_navigation': True,  # Sticky header
    'navigation_depth': 3,  # Sidebar navigation depth
}

# Link to custom CSS file
html_static_path = ['_static']
html_css_files = [
    'custom.css',  # Link to custom styles
]

# -- Options for HTMLHelp output ---------------------------------------------
htmlhelp_basename = 'ResearchTrack2-Assignment-1doc'

# -- Options for LaTeX output ------------------------------------------------
latex_documents = [
    (master_doc, 'ResearchTrack2-Assignment-1.tex', 'ResearchTrack2-Assignment-1 Documentation', 'NimaiDey', 'manual'),
]

# -- Options for manual page output ------------------------------------------
man_pages = [
    (master_doc, 'researchtrack2-assignment-1', 'ResearchTrack2-Assignment-1 Documentation',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------
texinfo_documents = [
    (master_doc, 'ResearchTrack2-Assignment-1', 'ResearchTrack2-Assignment-1 Documentation',
     author, 'ResearchTrack2-Assignment-1', 'One line description of project.',
     'Miscellaneous'),
]

# -- Options for Epub output -------------------------------------------------
epub_title = project
epub_exclude_files = ['search.html']

# -- Extension configuration -------------------------------------------------
todo_include_todos = True

