# Contributing

Thank you for helping. A few rules keep this small project dependable.

## How changes get in

* Open an issue first for anything larger than a small fix, so the approach
  can be agreed before you spend time on it.
* Fork, make a branch, and open a pull request against `main`. Nobody pushes
  to `main` directly, maintainers included.
* Every pull request is reviewed and approved by a QuantaByte maintainer
  before it is merged. Please be patient: this is maintained alongside
  other work.

## What a pull request needs

* **Tested on a real board.** Say which RDK X5, which RDK OS version
  (`cat /etc/version`), which `hobot-camera` version
  (`dpkg -l hobot-camera`), which camera module, and what you ran.
* **One thing per pull request.** A fix and an unrelated clean-up are two
  pull requests.
* **Plain explanations.** Say what was wrong, how you found it, and how you
  know it is fixed. Numbers are better than adjectives.
* **Your own work, under this licence.** By contributing you agree that your
  contribution is licensed GPL-2.0-only, and you confirm you have the right
  to submit it. Sign your commits off (`git commit -s`), which states exactly
  that (the Developer Certificate of Origin, https://developercertificate.org).
* Do not paste code from datasheets or other drivers unless its licence
  allows it under GPL-2.0, and say where it came from.

## Reporting a problem

Use the issue form. The most useful things to include are the output of
`journalctl -u imx708-csi -n 50 --no-pager` and of
`python3 examples/imx708_cam.py --info`.

## Conduct

Be kind and be specific. Disagree about the code, not the person.
