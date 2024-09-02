from ... import *
from . import BlackmagicApplet


class BlackmagicAppletTestCase(GlasgowAppletTestCase, applet=BlackmagicApplet):
    @synthesis_test
    def test_build(self):
        self.assertBuilds()
