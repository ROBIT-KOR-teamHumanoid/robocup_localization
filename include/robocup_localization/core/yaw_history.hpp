#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

namespace robocup_localization::core
{

  enum class YawLookup
  {
    Ok,
    TooOld,
    Empty,
  };

  // Recent IMU yaw samples, so a camera frame can be rotated with the yaw it was taken at.
  class YawHistory
  {
  public:
    explicit YawHistory(std::int64_t span_ns = 1000000000LL)
        : span_ns_(span_ns)
    {
    }

    void setSpan(std::int64_t span_ns)
    {
      span_ns_ = span_ns;
      trim();
    }

    void push(std::int64_t time_ns, double yaw_deg)
    {
      // Out-of-order samples would break the sorted search, so they are dropped.
      if (!samples_.empty() && time_ns <= samples_.back().time_ns)
      {
        if (time_ns == samples_.back().time_ns)
        {
          samples_.back().yaw_deg = yaw_deg;
        }
        return;
      }

      samples_.push_back(Sample{time_ns, yaw_deg});
      trim();
    }

    YawLookup lookup(std::int64_t time_ns, double& yaw_deg) const
    {
      if (samples_.empty())
      {
        return YawLookup::Empty;
      }

      if (time_ns < samples_.front().time_ns)
      {
        return YawLookup::TooOld;
      }

      // Newer than the last sample: hold it rather than extrapolate.
      if (time_ns >= samples_.back().time_ns)
      {
        yaw_deg = samples_.back().yaw_deg;
        return YawLookup::Ok;
      }

      const auto after = std::upper_bound(
          samples_.begin(), samples_.end(), time_ns,
          [](std::int64_t t, const Sample& sample)
          {
            return t < sample.time_ns;
          });
      const auto before = after - 1;

      const double span = static_cast<double>(after->time_ns - before->time_ns);
      const double ratio = static_cast<double>(time_ns - before->time_ns) / span;
      const double delta = shortestDelta(before->yaw_deg, after->yaw_deg);

      yaw_deg = wrapLikeSamples(
          before->yaw_deg + ratio * delta, before->yaw_deg, after->yaw_deg);
      return YawLookup::Ok;
    }

    std::size_t size() const
    {
      return samples_.size();
    }

  private:
    struct Sample
    {
      std::int64_t time_ns;
      double yaw_deg;
    };

    static double shortestDelta(double from_deg, double to_deg)
    {
      double delta = std::fmod(to_deg - from_deg, 360.0);
      if (delta > 180.0)
      {
        delta -= 360.0;
      }
      else if (delta < -180.0)
      {
        delta += 360.0;
      }
      return delta;
    }

    // Keep the IMU's own range, (-180, 180] or [0, 360), when interpolating across the seam.
    static double wrapLikeSamples(double yaw_deg, double a_deg, double b_deg)
    {
      if (a_deg < 0.0 || b_deg < 0.0)
      {
        while (yaw_deg > 180.0)
        {
          yaw_deg -= 360.0;
        }
        while (yaw_deg <= -180.0)
        {
          yaw_deg += 360.0;
        }
      }
      else if (a_deg > 180.0 || b_deg > 180.0)
      {
        while (yaw_deg >= 360.0)
        {
          yaw_deg -= 360.0;
        }
        while (yaw_deg < 0.0)
        {
          yaw_deg += 360.0;
        }
      }
      return yaw_deg;
    }

    void trim()
    {
      while (samples_.size() > 2 &&
             samples_.back().time_ns - samples_.front().time_ns > span_ns_)
      {
        samples_.pop_front();
      }
    }

    std::int64_t span_ns_;
    std::deque<Sample> samples_;
  };

}
