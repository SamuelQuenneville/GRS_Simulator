/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "spectralWindField.h"

SpectralWindField::SpectralWindField(const windProperties& parameters)
    : m_params(parameters)
    , m_rng(parameters.seed)
{
    if (m_params.turbulenceNumberMode % 2 != 0) {
        throw std::runtime_error("SpectralWindField: numberModes must be even.");
    }

    m_generateModes();
    m_calibrate();
}

grs::Vec3d  SpectralWindField::getTurbulence(const grs::Vec3d& position, const grs::Vec3d& advectionVelocity, const double time) const {
    grs::Vec3d  wind = grs::Vec3d ::zeros();

    for (const auto&[k, e1, e2, amplitude, phase] : m_fourierModes) {
        const double theta = grs::dot(k, position) - grs::dot(k, advectionVelocity) * time + phase;

        const double c = std::cos(theta);
        const double s = std::sin(theta);

        wind += amplitude * (e1 * c + e2 * s);
    }

    return grs::cwiseProduct(wind, m_scaleFactor);
}

void SpectralWindField::m_generateModes() {
    m_fourierModes.clear();
    m_fourierModes.reserve(m_params.turbulenceNumberMode);

    // Number of k/-k pairs
    const std::size_t nPairs = m_params.turbulenceNumberMode / 2;

    // Von Karman constant
    const double C = 55.0 / (9.0 * std::sqrt(M_PI)) * std::tgamma(5.0 / 6.0) / std::tgamma(1.0 / 3.0);

    // Logarithmically spaced wavelengths
    std::vector<double> lambda(nPairs);

    const double logMin = std::log10(m_params.turbulenceLambdaMin);
    const double logMax = std::log10(m_params.turbulenceLambdaMax);

    for(std::size_t i=0 ; i<nPairs ; i++) {
        const double alpha = static_cast<double>(i) / static_cast<double>(nPairs-1);

        lambda[i] = std::pow(10.0, logMin + alpha*(logMax-logMin));
    }

    // Convert to wave numbers
    std::vector<double> k(nPairs);

    for(std::size_t i=0 ; i<nPairs ; i++) {
        k[i] = 2.0 * M_PI / lambda[i];
    }

    std::sort(k.begin(),k.end());

    // dk
    std::vector<double> dk(nPairs);

    dk.front() = k[1]-k[0];
    dk.back() = k.back()-k[k.size()-2];

    for(std::size_t i=1 ; i<nPairs-1 ; i++) {
        dk[i]= 0.5 * (k[i+1]-k[i-1]);
    }

    //--------------------------------------------------------------
    // Generate all mode pairs
    //--------------------------------------------------------------
    for(std::size_t i=0 ; i<nPairs ; i++) {
        const double kmag = k[i];

        // Von Karman spectrum
        const double x = kmag * m_params.turbulenceLengthScale;

        const double E = C * m_params.turbulenceLengthScale * std::pow(x,4.0) / std::pow(1.0+x*x,17.0/6.0);

        const double amplitude = std::sqrt(2.0 * E * dk[i]);

        // Random wave direction
        grs::Vec3d khat = m_randomUnitVector();

        // Polarization basis
        grs::Vec3d e1{};

        if (std::abs(khat[0]) < 0.9) {
            e1 = grs::cross(khat, {1,0,0});
        }
        else {
            e1 = grs::cross(khat, {0,1,0});
        }

        e1 = grs::normalize(e1);

        const grs::Vec3d  e2 = grs::normalize(grs::cross(khat, e1));

        // Random phase
        const double phi = m_uniform(m_rng);

        // Positive mode
        fourierMode positive{};

        positive.k = kmag*khat;
        positive.e1 = e1;
        positive.e2 = e2;
        positive.amplitude = amplitude;
        positive.phase = phi;

        m_fourierModes.push_back(positive);

        // Negative mode
        fourierMode negative = positive;

        negative.k = -positive.k;

        m_fourierModes.push_back(negative);
    }
}

void SpectralWindField::m_calibrate()
{
    grs::Vec3d variance = grs::Vec3d::zeros();

    // One iteration per k/-k pair
    for (std::size_t i = 0; i < m_fourierModes.size(); i += 2) {
        const fourierMode& mode = m_fourierModes[i];

        const double c = std::cos(mode.phase);
        const double s = std::sin(mode.phase);

        // V = e1*cos(phi) + e2*sin(phi)
        grs::Vec3d V = mode.e1 * c + mode.e2 * s;

        variance += 2.0 * mode.amplitude * mode.amplitude * grs::cwiseProduct(V, V);
    }

    // RMS of the unscaled field
    m_rmsUncalibrated = grs::cwiseSqrt(variance);

    // Prevent divide-by-zero
    constexpr double eps = 1e-12;

    for (int i = 0; i < 3; ++i) {
        if (m_rmsUncalibrated[i] < eps) {
            m_rmsUncalibrated[i] = eps;
        }
    }

    // Scale factors
    m_scaleFactor = grs::cwiseQuotient(grs::toVector<double,3>(m_params.sigma), m_rmsUncalibrated);
}

grs::Vec3d SpectralWindField::m_randomUnitVector() {
    grs::Vec3d v = grs::Vec3d::zeros();

    do {
        v[0] = m_normal(m_rng);
        v[1] = m_normal(m_rng);
        v[2] = m_normal(m_rng);

    } while(v.normSquared() < 1e-12);

    return grs::normalize(v);
}