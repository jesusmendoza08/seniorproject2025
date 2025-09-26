#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>

// =================================================================
// 1. HELPER FUNCTION: FIR Filter
// =================================================================
// Simulates a physical path (Primary or Secondary)
double firFilter(const std::vector<double>& inputBuffer, const std::vector<double>& pathCoeffs) {
    double output = 0.0;
    for (size_t i = 0; i < pathCoeffs.size(); ++i) {
        if (i < inputBuffer.size()) {
            output += pathCoeffs[i] * inputBuffer[i];
        }
    }
    return output;
}

// =================================================================
// 2. FXLMS CLASS
// =================================================================
class FXLMS {
private:
    int filterLen;
    double mu;
    std::vector<double> w;            // adaptive filter coefficients
    std::vector<double> xBuffer;      // reference input buffer
    std::vector<double> xFiltBuffer;  // filtered-x buffer
    std::vector<double> secPathEst;   // estimated secondary path (S_hat)

public:
    FXLMS(int filterLen, double mu, const std::vector<double>& secPathEst)
        : filterLen(filterLen), mu(mu), w(filterLen, 0.0),
          xBuffer(filterLen, 0.0), xFiltBuffer(filterLen, 0.0),
          secPathEst(secPathEst) {}

    // Step 1: Compute output y(n) given reference x(n)
    double computeOutput(double x) {
        // Shift reference buffer
        for (int i = filterLen - 1; i > 0; --i)
            xBuffer[i] = xBuffer[i - 1];
        xBuffer[0] = x;

        // Filter x through secondary path estimate (S_hat)
        double xFiltered = firFilter(xBuffer, secPathEst);

        // Shift filtered-x buffer
        for (int i = filterLen - 1; i > 0; --i)
            xFiltBuffer[i] = xFiltBuffer[i - 1];
        xFiltBuffer[0] = xFiltered;

        // Compute adaptive filter output
        double y = 0.0;
        for (int i = 0; i < filterLen; i++)
            y += w[i] * xBuffer[i];

        return y;
    }

    // Step 2: Update filter weights with current error e(n)
    void updateWeights(double e) {
        for (int i = 0; i < filterLen; i++)
            w[i] += mu * e * xFiltBuffer[i];
    }

    const std::vector<double>& getWeights() const {
        return w;
    }
};

// =================================================================
// 3. MAIN SIMULATION
// =================================================================
int main() {
    // --- ANC Parameters ---
    int filterLen = 32;        // filter length
    double mu = 0.005;         // step size

    // --- REAL-WORLD PATHS ---
    std::vector<double> primaryPath = {1.0, 0.5, 0.2, 0.1, 0.05}; // P
    std::vector<double> secondaryPath = {0.8, 0.6, 0.3, 0.15};   // S

    // --- Estimated Secondary Path (S_hat) ---
    std::vector<double> secPathEst = {0.85, 0.55, 0.35, 0.1}; // slightly mismatched

    FXLMS fx(filterLen, mu, secPathEst);

    // Buffers
    std::vector<double> xBufferP(
        std::max((size_t)filterLen, std::max(primaryPath.size(), secondaryPath.size())), 0.0);
    std::vector<double> yBufferS(
        std::max((size_t)filterLen, secondaryPath.size()), 0.0);

    // Input reference signal
    std::vector<double> xSignal = {
        1.0, 0.5, -0.2, 0.3, -0.7, 0.6, 0.1, -0.3, 0.4, -0.5,
        0.9, 0.4, -0.1, 0.2, -0.6, 0.55, 0.05, -0.25, 0.35, -0.45
    };

    // --- Simulation Loop ---
    std::cout << "n | x(n) | d(n) (Primary Noise) | y(n) (Anti-Noise) | e(n) (Error)\n";
    std::cout << "--|------|----------------------|-------------------|-------------------\n";

    for (size_t n = 0; n < xSignal.size(); n++) {
        double x = xSignal[n];

        // 1. Primary noise: d(n) = P * x(n)
        for (int i = xBufferP.size() - 1; i > 0; --i)
            xBufferP[i] = xBufferP[i - 1];
        xBufferP[0] = x;
        double d = firFilter(xBufferP, primaryPath);

        // 2. Compute anti-noise y(n)
        double y = fx.computeOutput(x);

        // 3. Secondary path response: y_filt_S = S * y(n)
        for (int i = yBufferS.size() - 1; i > 0; --i)
            yBufferS[i] = yBufferS[i - 1];
        yBufferS[0] = y;
        double y_filt_S = firFilter(yBufferS, secondaryPath);

        // 4. Error: e(n) = d(n) - S*y(n)
        double e = d - y_filt_S;

        // 5. Update adaptive filter weights
        fx.updateWeights(e);

        std::cout << n << " | "
                  << x << " | "
                  << d << " | "
                  << y << " | "
                  << e << "\n";
    }

    // --- Final Weights ---
    auto w = fx.getWeights();
    std::cout << "\nFinal Weights (W): ";
    for (auto wi : w) std::cout << wi << " ";
    std::cout << "\n(These should approximate an inverse of P, filtered by S_hat.)\n";

    return 0;
}