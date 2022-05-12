#include "gmmloc/gmm/gaussian_mixture.h"

#include <algorithm>
#include <numeric>
#include <vector>

#include "gmmloc/gmm/gmm_utils.h"

#include "gmmloc/config.h"

namespace gmmloc
{

  using namespace std;

  inline cv::Vec3b makeJet3B(float id)
  {
    if (id <= 0)
      return cv::Vec3b(128, 0, 0);
    if (id >= 1)
      return cv::Vec3b(0, 0, 128);

    int icP = (id * 8);
    float ifP = (id * 8) - icP;

    if (icP == 0)
      return cv::Vec3b(255 * (0.5 + 0.5 * ifP), 0, 0);
    if (icP == 1)
      return cv::Vec3b(255, 255 * (0.5 * ifP), 0);
    if (icP == 2)
      return cv::Vec3b(255, 255 * (0.5 + 0.5 * ifP), 0);
    if (icP == 3)
      return cv::Vec3b(255 * (1 - 0.5 * ifP), 255, 255 * (0.5 * ifP));
    if (icP == 4)
      return cv::Vec3b(255 * (0.5 - 0.5 * ifP), 255, 255 * (0.5 + 0.5 * ifP));
    if (icP == 5)
      return cv::Vec3b(0, 255 * (1 - 0.5 * ifP), 255);
    if (icP == 6)
      return cv::Vec3b(0, 255 * (0.5 - 0.5 * ifP), 255);
    if (icP == 7)
      return cv::Vec3b(0, 0, 255 * (1 - 0.5 * ifP));
    return cv::Vec3b(255, 255, 255);
  }

  GMM::GMM(const GaussianComponents &components)
      : num_(components.size()), components_(components)
  {
    components2d_.resize(num_);

    fill(components2d_.begin(), components2d_.end(), nullptr);

    vector<int> v;

    int count_deg = 0;
    for (auto &comp : components_)
    {
      if (comp->is_degenerated)
      {
        count_deg++;
      }
    }

    LOG(INFO) << "#deg: " << count_deg
              << " #non_deg: " << components_.size() - count_deg;

    for (auto &comp : components_)
    {
      int count_comp = 0;

      for (auto &comp2 : components_)
      {
        if (comp == comp2)
          continue;

        double dist = GMMUtility::BHCoefficient(*comp, *comp2);
        // cout << dist << endl;
        if (dist < gmmmap::neighbor_dist_thresh)
        {
          count_comp++;
          GaussianComponent::NeighbourInfo info;
          info.ptr = comp2;
          info.dist = dist;
          comp->nbs_.push_back(info);
        }
      }
    }

    // double sum = std::accumulate(v.begin(), v.end(), 0.0);
    // double mean = sum / v.size();

    // double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    // double stdev = std::sqrt(sq_sum / v.size() - mean * mean);

    // LOG(INFO) << "mean: " << mean << " std: " << stdev;

    // exit(0);

    this->buildKDTree();
  }

  inline bool compareDepth(const GaussianComponent2d *a,
                           const GaussianComponent2d *b)
  {
    return a->proj_d_ < b->proj_d_;
  };

  void GMM::renderViewProb(const Quaterniond &rot_c_w, const Vector3d &t_c_w)
  {
    LOG(INFO) << "rendering view";

    components2d_.clear();

    std::vector<std::pair<double, GaussianComponent2d *>> depth_;

    for (size_t idx = 0; idx != components_.size(); idx++)
    {
      /* code */
      auto &&mu = components_[idx]->mean();

      // STEP.0 check view cos
      bool check_view_cos = true;
      double view_cos_thresh = cos(78.0 * M_PI / 180.0);
      double view_cos;
      if (check_view_cos && components_[idx]->is_degenerated)
      {
        // if (check_view_cos) {

        const Vector3d t_w_c = -(rot_c_w.inverse() * t_c_w);

        const Vector3d po = (mu - t_w_c).normalized();

        auto &&axis = components_[idx]->axis_.col(0);

        view_cos = abs(po.dot(axis));

        // cout << idx << endl;
        // cout << view_cos << " angle: " << acos(view_cos) / M_PI * 180.0 <<
        // endl;
        if (view_cos < view_cos_thresh)
          continue;
      }

      // STEP. projection

      auto g2d = GMMUtility::projectGaussian(*components_[idx], camera_, rot_c_w, t_c_w);

      if (g2d)
      {
        // STEP. check cov
        bool check_cov_2d = true;
        double cov_2d_thresh = 4.0;
        if (check_cov_2d)
        {
          auto &&scale_2d = g2d->scale();
          if (scale_2d.x() < cov_2d_thresh && scale_2d.y() < cov_2d_thresh)
            continue;
        }

        g2d->id_ = idx;
        g2d->parent_ = components_[idx];

        Eigen::Vector3d pc = rot_c_w * mu + t_c_w;

        // g2d->parent_ = components_[idx];
        g2d->proj_d_ = pc.z();

        // STEP. check near far
        bool check_depth = true;
        double depth_thresh = 0.8;
        // bool replaced = false;
        if (check_depth && !components2d_.empty())
        {
          size_t min_idx;
          double min_dist = numeric_limits<double>::max();
          for (size_t idx = 0; idx < components2d_.size(); idx++)
          {
            double dist = GMMUtility::BHCoefficient(*components2d_[idx], *g2d);
            if (dist < min_dist)
            {
              min_dist = dist;
              min_idx = idx;
            }
          }
          auto &min_ref = components2d_[min_idx];
          if (min_dist < depth_thresh)
          {
            if (g2d->proj_d_ < min_ref->proj_d_)
            {
              min_ref = g2d;
            }
          }
          else
          {
            components2d_.push_back(g2d);
          }

          // if (!replaced)
        }
        else
        {
          components2d_.push_back(g2d);
        }

        // depth_.push_back(std::make_pair(g2d->proj_d_, g2d));
      }
    }

    //   if (sort_by_depth) {
    sort(components2d_.begin(), components2d_.end(),
         [](const GaussianComponent2d::Ptr &a, const GaussianComponent2d::Ptr &b)
             -> bool
         { return a->proj_d_ > b->proj_d_; });
  }

  void GMM::renderViewCovarianceCheck(const Quaterniond &rot_c_w,
                                      const Vector3d &t_c_w)
  {
    LOG(INFO) << "rendering view";

    components2d_.clear();

    std::vector<std::pair<double, GaussianComponent2d::Ptr>> depth_;

    // std::vector<double, GaussianComponent2d> depth_;

    for (size_t idx = 0; idx != components_.size(); idx++)
    {
      /* code */
      auto &&mu = components_[idx]->mean();

      // STEP.0 check view cos
      bool check_view_cos = false;
      double view_cos_thresh = cos(78.0 * M_PI / 180.0);
      double view_cos;
      if (check_view_cos && components_[idx]->is_degenerated)
      {
        // if (check_view_cos) {

        const Vector3d t_w_c = -(rot_c_w.inverse() * t_c_w);

        const Vector3d po = (mu - t_w_c).normalized();

        auto &&axis = components_[idx]->axis_.col(0);

        view_cos = abs(po.dot(axis));

        // cout << idx << endl;
        // cout << view_cos << " angle: " << acos(view_cos) / M_PI * 180.0 <<
        // endl;
        if (view_cos < view_cos_thresh)
          continue;
      }

      // STEP. projection

      auto g2d = GMMUtility::projectGaussian(*components_[idx], camera_, rot_c_w, t_c_w);

      if (g2d)
      {
        // STEP. check cov
        bool check_cov_2d = false;
        double cov_2d_thresh = 4.0;
        if (check_cov_2d)
        {
          auto &&scale_2d = g2d->scale();
          if (scale_2d.x() < cov_2d_thresh && scale_2d.y() < cov_2d_thresh)
            continue;
        }

        g2d->id_ = idx;
        g2d->parent_ = components_[idx];

        Eigen::Vector3d pc = rot_c_w * mu + t_c_w;

        // g2d->parent_ = components_[idx];
        g2d->proj_d_ = pc.z();

        // depth_.push_back(std::make_pair(g2d->proj_d_, g2d));
        components2d_.push_back(g2d);
      }
    }

    // if (sort_by_depth)
    {
      sort(components2d_.begin(), components2d_.end(),
           [](const GaussianComponent2d::Ptr &a,
              const GaussianComponent2d::Ptr &b) -> bool
           {
             return a->proj_d_ < b->proj_d_;
           });
    }

    // for (auto &&dg : depth_) {
    //   components2d_.push_back(dg.second);
    // }
  }

  /**
   * @brief 将先验的GMM投影到keyframe2D像素平面，并根据相机视野范围、遮挡情况进行GMM筛选
   *
   * @param rot_c_w 旋转向量
   * @param t_c_w 平移向量
   */
  void GMM::renderView(const Quaterniond &rot_c_w, const Vector3d &t_c_w)
  {
    LOG(INFO) << "rendering view";

    components2d_.clear(); // 2D的GMM

    std::vector<std::pair<double, GaussianComponent2d *>> depth_;

    // - 遍历所有的GMM
    for (size_t idx = 0; idx != components_.size(); idx++)
    {
      /* code */
      auto &&mu = components_[idx]->mean(); // GAUSS的均值

      // - 筛选在相机视野范围内的3DGMM
      //* 对应论文 III. METHOD D. Projection of the GMM Map 的计算相机视野的部分
      // STEP.0 check view cos
      bool check_view_cos = true;
      double view_cos_thresh = cos(78.0 * M_PI / 180.0);
      double view_cos;
      if (check_view_cos && components_[idx]->is_degenerated) //检查视场角&&GAUSS没退化
      {
        // if (check_view_cos) {
        const Vector3d t_w_c = -(rot_c_w.inverse() * t_c_w); // world坐标系下camera的位置

        const Vector3d po = (mu - t_w_c).normalized(); //从相机指向GMM均值的向量的单位化

        auto &&axis = components_[idx]->axis_.col(0); // GMM投影退化方向的轴，就是相机光轴方向被，毕竟后面是投影到keyframe的2D平面

        view_cos = abs(po.dot(axis)); //计算相机的视场角的cos值

        // cout << idx << endl;
        // cout << view_cos << " angle: " << acos(view_cos) / M_PI * 180.0 <<
        // endl;
        if (view_cos < view_cos_thresh) //判断投影的2D的GMM超出视野范围
          continue;
      }

      // STEP. projection
      //  - GMM投影到keyframe像素坐标系下的均值和方差，不可视的情况会返回nullptr
      auto g2d = GMMUtility::projectGaussian(*components_[idx], camera_, rot_c_w, t_c_w);

      if (g2d)
      {
        //  - 检测协方差是否符合阈值
        //* 对应论文 III. METHOD D. Projection of the GMM Map 的GMM判断协方差阈值部分
        // STEP. check cov
        bool check_cov_2d = true;
        double cov_2d_thresh = 4.0;
        if (check_cov_2d)
        {
          //分解协方差，协方差分量小于阈值则舍去
          auto &&scale_2d = g2d->scale();
          if (scale_2d.x() < cov_2d_thresh && scale_2d.y() < cov_2d_thresh)
            continue;
        }

        g2d->id_ = idx;                  //对应的GMMid
        g2d->parent_ = components_[idx]; //对应的3D的GMM

        Eigen::Vector3d pc = rot_c_w * mu + t_c_w; //投影到keyframe坐标系下GMM均值

        // g2d->parent_ = components_[idx];
        g2d->proj_d_ = pc.z(); //更新深度

        //  - 遍历所有的投影完成非GMM,计算当前GMM与之前的GMM分布的距离，判定遮挡情况，取非遮挡的GMM投影
        //* 对应论文 III. METHOD D. Projection of the GMM Map 的GMM遮挡部分
        // STEP. check near far
        bool check_depth = true;
        double depth_thresh = 0.8;
        // bool replaced = false;
        if (check_depth && !components2d_.empty())
        {
          size_t min_idx;
          double min_dist = numeric_limits<double>::max();
          for (size_t idx = 0; idx < components2d_.size(); idx++)
          {
            double dist = GMMUtility::BHCoefficient(*components2d_[idx], *g2d);
            if (dist < min_dist)
            {
              min_dist = dist;
              min_idx = idx;
            }
          }
          auto &min_ref = components2d_[min_idx];
          if (min_dist < depth_thresh) //如果GMM距离小于阈值则判定为发生遮挡
          {
            //保存距离相机最近的一个GMM，最近的没发生遮挡
            if (g2d->proj_d_ < min_ref->proj_d_) //判断深度取最近点
            {
              min_ref = g2d; // min_ref取[当前GMM][与当前GMM距离最近的GMM]中z深度最小的一个
              //投影到2D的GMM，距离最近的就产生遮挡了，取z深度最小的就是取距离相机近的，不好遮挡
            }
          }
          else
          {
            components2d_.push_back(g2d); //没有遮挡直接保存
          }

          // if (!replaced)
        }
        else
        {
          /* code */
          components2d_.push_back(g2d); //第一个GMM一定没有遮挡也直接保存
        }

        // depth_.push_back(std::make_pair(g2d->proj_d_, g2d));
      }
    }

    //   if (sort_by_depth) {
    //按照z深度对2D进行排序
    sort(components2d_.begin(), components2d_.end(),
         [](const GaussianComponent2d::Ptr &a, const GaussianComponent2d::Ptr &b)
             -> bool
         { return a->proj_d_ > b->proj_d_; });
    // }
    // sort(depth_.begin(), depth_.end());

    // for (auto &&dg : depth_) {
    //   components2d_.push_back(dg.second);
    // }
  }

  void GMM::renderView(const Quaterniond &rot_c_w, const Vector3d &t_c_w,
                       std::vector<uint32_t> indices, bool sort_by_depth)
  {

    components2d_.clear();

    for (auto &&idx : indices)
    {
      /* code */
      auto g2d =
          GMMUtility::projectGaussian(*components_[idx], camera_, rot_c_w, t_c_w);

      if (g2d)
      {
        auto &&mu = components_[idx]->mean();

        auto pc = rot_c_w * mu + t_c_w;

        g2d->parent_ = components_[idx];
        g2d->proj_d_ = pc.z();

        components2d_.push_back(g2d);
      }
    }

    if (sort_by_depth)
    {
      sort(components2d_.begin(), components2d_.end(),
           [](const GaussianComponent2d::Ptr &a,
              const GaussianComponent2d::Ptr &b) -> bool
           {
             return a->proj_d_ > b->proj_d_;
           });
    }
  }

  void GMM::visualize2d(cv::Mat &viz_img) const
  {
    //   cv::Mat viz_img(480, 640, CV_8UC3, cv::Scalar(0));

    // float size_ = components2d_.size();
    // int ms_wk = -1;

    // Eigen::Vector2d mean_avg = Eigen::Vector2d::Zero();
    // Eigen::Matrix2d cov_avg = Eigen::Matrix2d::Zero();
    // Eigen::Matrix2d cov_A = Eigen::Matrix2d::Zero();

    // for (size_t i = 0; i < components2d_.size(); i++) {

    //   auto &&g2d = components2d_[i];
    //   if (!g2d)
    //     continue;

    //   cv::Mat bk_img = viz_img.clone();

    //   // auto &&mu = g2d->mean();
    //   // auto &&theta = g2d->theta();
    //   // auto &&scale = g2d->scale();
    //   // auto &&cov = g2d->cov();
    // }
  }

  void GMM::searchCorrespondence(const std::vector<Feature> &kpts,
                                 GaussianComponents2d &comps)
  {
    // preprocessing
    comps.clear();
    comps.resize(kpts.size(), nullptr);

    // build index
    FLANNPoints2d pts(components2d_);
    auto kdtree =
        new KDTreePoints2d(2, pts, nanoflann::KDTreeSingleIndexAdaptorParams(5));

    kdtree->buildIndex();

    const bool check_mdist2 = true;
    const double mdist2_thresh = 9.0;

    // searching
    const int num_results = 4;
    for (size_t i = 0; i < kpts.size(); i++)
    {

      const auto &kp = kpts[i].uv;

      double pt_query[2] = {kp.x(), kp.y()};

      std::vector<size_t> ret_index(num_results);
      std::vector<double> out_dist_sqr(num_results);
      // kdtree->knnSearch(pt_query, num_nearest, )
      auto num = kdtree->knnSearch(&pt_query[0], num_results, &ret_index[0],
                                   &out_dist_sqr[0]);

      // In case of less points in the tree than requested:
      ret_index.resize(num);
      out_dist_sqr.resize(num);
      if (num)
      {
        if (check_mdist2)
        {
          size_t min_idx;
          double min_dist = numeric_limits<double>::max();
          for (size_t reti = 0; reti < ret_index.size(); reti++)
          {
            auto dist = components2d_[ret_index[reti]]->MDist2(
                Eigen::Map<Vector2d>(pt_query));
            if (dist < min_dist)
            {
              min_dist = dist;
              min_idx = ret_index[reti];
            }
          }
          if (min_dist < mdist2_thresh)
            comps[i] = components2d_[min_idx];
        }
        else
        {
          comps[i] = components2d_[ret_index[0]];
        }
      }
    }
  }

  /**
   * @brief 进行KDtree最近邻搜索先验地图的GMM与keyframe特征点进行数据关联
   * ?为什么是构建GMM的tree，反过来效果怎么样啊
   * @param kpts 当前keyframe中的特征点
   * @param comps 2DGMM数据关联结果
   * @param num 默认参数5
   */
  void GMM::searchCorrespondence(const std::vector<Feature> &kpts,
                                 std::vector<GaussianComponents2d> &comps,
                                 int num)
  {

    // preprocessing
    comps.clear();
    comps.resize(kpts.size(), GaussianComponents2d());

    // build index
    //构建GMM的kdtree索引
    FLANNPoints2d pts(components2d_);                                               // 2DGMM像素坐标
    auto kdtree = new KDTreePoints2d(2,                                             // 2维kdtree
                                     pts,                                           //输入数据
                                     nanoflann::KDTreeSingleIndexAdaptorParams(5)); //?叶子节点最大size

    kdtree->buildIndex();

    const bool check_mdist2 = true;
    const double mdist2_thresh = 9.0;

    // searching
    const int num_results = num; // kdtree搜索保存最好的结果数量
    for (size_t i = 0; i < kpts.size(); i++)
    {

      const auto &kp = kpts[i].uv;

      double pt_query[2] = {kp.x(), kp.y()};

      std::vector<size_t> ret_index(num_results);
      std::vector<double> out_dist_sqr(num_results);
      // kdtree->knnSearch(pt_query, num_nearest, )
      //最近邻搜索寻找特征点最近的GMM
      auto num_final = kdtree->knnSearch(&pt_query[0], num_results, &ret_index[0], &out_dist_sqr[0]); //计算后符合要求的结果数量

      // In case of less points in the tree than requested:
      ret_index.resize(num_final);
      out_dist_sqr.resize(num_final);
      comps[i].clear();
      if (num_final)
      {
        //遍历最近邻选出的所有GMM，将符合条件的特征点关联到GMM
        for (auto &&idx : ret_index) //保存最近邻搜索的特征点
        {
          if (check_mdist2) //是否检测2阶中心距阈值
          {
            auto dist = components2d_[idx]->MDist2(Eigen::Map<Vector2d>(pt_query));
            if (dist < mdist2_thresh)
            {
              comps[i].push_back(components2d_[idx]);
            }
          }
          else
          {
            comps[i].push_back(components2d_[idx]);
          }
        }
      }
    }
  }

  void GMM::buildKDTree()
  {

    pts3d_ = new FLANNPoints3d(components_);
    kdtree3d_ = new KDTreePoints3d(3, *pts3d_,
                                   nanoflann::KDTreeSingleIndexAdaptorParams(5));

    kdtree3d_->buildIndex();
  }

  void GMM::queryPoint(const Eigen::Vector3d &pt, vector<int> &res)
  {
    if (!kdtree3d_)
    {
      LOG(ERROR) << "kdtree not init";
      return;
    }

    double pt_query[3] = {pt.x(), pt.y(), pt.z()};

    int num_results = 5;
    std::vector<size_t> ret_index(num_results);
    std::vector<double> out_dist_sqr(num_results);

    auto num_final = kdtree3d_->knnSearch(&pt_query[0], num_results,
                                          &ret_index[0], &out_dist_sqr[0]);

    res.clear();
    if (num_final)
    {
      // size_t min_idx;
      double min_dist = numeric_limits<double>::max();

      for (size_t reti = 0; reti < ret_index.size(); reti++)
      {
        auto dist = components_[ret_index[reti]]->MDist2(pt);

        if (dist < min_dist)
        {
          min_dist = dist;
          // min_idx = ret_index[reti];
        }
      }

      res.push_back(ret_index[0]);
    }
  }

  GMM::~GMM()
  {
    delete kdtree3d_;
    delete pts3d_;
  }

} // namespace gmmloc
