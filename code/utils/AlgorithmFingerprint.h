#ifndef HGASP_UTILS_ALGORITHM_FINGERPRINT_H_
#define HGASP_UTILS_ALGORITHM_FINGERPRINT_H_

#include "Version.h"

#include <cstdlib>
#include <string>

namespace hgasp {

inline constexpr const char* kFingerprintGitCommitEnvironment =
	"HGASP_FINGERPRINT_GIT_COMMIT";
inline constexpr const char* kFingerprintGitDirtyEnvironment =
	"HGASP_FINGERPRINT_GIT_DIRTY";
inline constexpr const char* kFingerprintResolvedConfigSha256Environment =
	"HGASP_FINGERPRINT_RESOLVED_CONFIG_SHA256";
inline constexpr const char* kFingerprintInstanceSha256Environment =
	"HGASP_FINGERPRINT_INSTANCE_SHA256";
struct AlgorithmFingerprint {
	std::string gitCommit;
	bool gitDirty = true;
	std::string resolvedConfigSha256;
	std::string instanceSha256;
	bool hardTwRouteLocalSearch = true;
};

inline std::string fingerprintEnvironmentValue(const char* const name) {
	const char* const value = std::getenv(name);
	return value == nullptr ? std::string() : std::string(value);
}

inline AlgorithmFingerprint runtimeAlgorithmFingerprint() {
	AlgorithmFingerprint result;
	result.gitCommit =
		fingerprintEnvironmentValue(kFingerprintGitCommitEnvironment);
	result.gitDirty = fingerprintEnvironmentValue(
		kFingerprintGitDirtyEnvironment) != "0";
	result.resolvedConfigSha256 = fingerprintEnvironmentValue(
		kFingerprintResolvedConfigSha256Environment);
	result.instanceSha256 = fingerprintEnvironmentValue(
		kFingerprintInstanceSha256Environment);
	return result;
}

}  // namespace hgasp

#endif  // HGASP_UTILS_ALGORITHM_FINGERPRINT_H_
