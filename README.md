# yarp-objectTracking
This is Yarp module to perform object tracking using OpenCV trackers and the KalmanFilter ensemble based (KF-EBT by Senna, Pedro and Drummond, Isabela Neves and Bastos, Guilherme Sousa)



_________________
# References

If you find it useful or use it in your research, please cite [ this paper](https://psenna.github.io/Papers/PID4960379.pdf).

~~~{yaml}
@InProceedings{SennaDrumBast:2017:ReEnTr,
               author = "Senna, Pedro and Drummond, Isabela Neves and Bastos, Guilherme Sousa",
                title = "Real-time ensemble-based tracker with Kalman filter",
            booktitle = "Electronic Proceedings of the 30th Conference on Graphics, Patterns and Images (SIBGRAPI'17)",
                 year = "2017",
         organization = "Conference on Graphics, Patterns and Images, 30.(SIBGRAPI)",
  conference-location = "Niter{\'o}i, RJ",
      conference-year = "Oct. 17-20, 2017",
             language = "en",
                  url = "http://urlib.net/sid.inpe.br/sibgrapi/2017/08.21.23.41"
}
~~~

Third-party software credits

Our tracker use several different trackers, and some files are provided by others and modified to work with our method.


The files trackers/kcf/kcf.h

		      /kcf.c

		      /complexmat.hpp

Are provided by Tomáš Vojíř kcf implementation

Source: https://github.com/vojirt/kcf


The files in trackers/ASMS are provided by Tomáš Vojíř ASMS implementation

Source: https://github.com/vojirt/asms


The files trackers/CBT/consensus/common.*

				/Consensus.*

				/fastcluster/*

Are provided by Georg Nebehay CppMT implementation

Source: https://github.com/gnebehay/CppMT


_________________
Copyright (c) 2017, Pedro Senna

Permission to use, copy, modify, and distribute this software for research purposes is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

__________________