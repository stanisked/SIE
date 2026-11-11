# SIEE end-of-day recovery record

The encrypted full-Pi-to-Drive plan from 2026-11-11 was superseded before
installation. The Drive account has a 10 GB limit; the Pi evidence alone is
approximately 12 GB, and the runtime environments another 6.6 GB. No restic
repository, Drive remote, Pi systemd timer or full Pi backup was confirmed.
The old implementation remains in Git history only. Do not deploy it from an
older commit.

Current rule: keep source history in `stanisked/SIE` and the change-controlled
`PROJECT_CONTEXT.md` in the SIE Active Projector Drive folder. At the end of
each working day, update the private
[SIEE Recovery Journal](https://drive.google.com/drive/folders/1roTCxsE8S8SehwLz2Q6-tnfxxB1J7yrK)
with verified Git commits, active Pi paths, purposes of important entrypoint
scripts, model/calibration identities, evidence locations and restoration
limits. Paths and hashes are an index; they cannot restore bytes that have no
off-Pi copy. Enumerate all tracked scripts from the pinned Git commit rather
than duplicating hundreds of paths in the journal.

Do not update `PROJECT_CONTEXT.md` automatically. Preserve its separate user
command rule. Mark any Pi state without a fresh observation as unverified.
